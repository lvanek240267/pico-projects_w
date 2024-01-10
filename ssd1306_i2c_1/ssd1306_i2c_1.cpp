/** 
 * Digital/Analog clock based on:
 * 
 * 1. Raspberry PI Pico W
 * 2. SSD1306 128x64 OLED I2C display
 * 3. NeoPixel circles Ø85 & Ø65, WS2812B
 * 
 * NeoPixel library: https://github.com/ForsakenNGS/Pico_WS2812
 * 
 * SSD1306:
 * GPIO PICO_SECOND_I2C_SDA_PIN (on Pico this is GP6 (pin 9)) -> SDA on display
 * GPIO PICO_SECOND_I2C_SCL_PIN (on Pico this is GP7 (pin 10)) -> SCL on display
 * 
 * Neopixels:
 * Data line (on Pico this is GP14 (pin 19))
 * Data line (on Pico this is GP15 (pin 20))
 * Note: in CMakeLists.txt is WiFi configuration
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/lock_core.h"
#include "hardware/i2c.h"
#include "ssd1306_font.h"
#include "pico/cyw43_arch.h"
#include "sd_card.h"
#include "ff.h"

/**
 * NTP Stuff, code borrowed from pico_examles/pico_w/ntp
 * 
 */
#include "lwip/dns.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

typedef struct NTP_T_ {
    ip_addr_t ntp_server_address;
    bool dns_request_sent;
    struct udp_pcb *ntp_pcb;
    absolute_time_t ntp_test_time;
    alarm_id_t ntp_resend_alarm;
} NTP_T;

#define NTP_SERVER "pool.ntp.org"
#define NTP_MSG_LEN 48
#define NTP_PORT 123
#define NTP_DELTA 2208988800 // seconds between 1 Jan 1900 and 1 Jan 1970
#define NTP_TEST_TIME (30 * 1000)
#define NTP_RESEND_TIME (10 * 1000)

time_t epoch = 0;

#include "pico/critical_section.h"
critical_section_t myLock;

#define BUTTON_DST 12 // Switch time to DST


/**
 * I2C OLED Stuff, code borrowed from pico_examles/i2c/ssd1306_i2c
 * 
 */

// Define the size of the display we have attached. This can vary, make sure you
// have the right size defined or the output will look rather odd!
// Code has been tested on 128x32 and 128x64 OLED displays
#define SSD1306_HEIGHT              64
#define SSD1306_WIDTH               128

// See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SSD1306_I2C_ADDR            _u(0x3D) // _u(0x3C)

// 400 is usual, but often these can be overclocked to improve display response.
// Tested at 1000 on both 32 and 84 pixel height devices and it worked.
#define SSD1306_I2C_CLK             400
//#define SSD1306_I2C_CLK             1000

// commands (see datasheet)
#define SSD1306_SET_MEM_MODE        _u(0x20)
#define SSD1306_SET_COL_ADDR        _u(0x21)
#define SSD1306_SET_PAGE_ADDR       _u(0x22)
#define SSD1306_SET_HORIZ_SCROLL    _u(0x26)
#define SSD1306_SET_SCROLL          _u(0x2E)

#define SSD1306_SET_DISP_START_LINE _u(0x40)

#define SSD1306_SET_CONTRAST        _u(0x81)
#define SSD1306_SET_CHARGE_PUMP     _u(0x8D)

#define SSD1306_SET_SEG_REMAP       _u(0xA0)
#define SSD1306_SET_ENTIRE_ON       _u(0xA4)
#define SSD1306_SET_ALL_ON          _u(0xA5)
#define SSD1306_SET_NORM_DISP       _u(0xA6)
#define SSD1306_SET_INV_DISP        _u(0xA7)
#define SSD1306_SET_MUX_RATIO       _u(0xA8)
#define SSD1306_SET_DISP            _u(0xAE)
#define SSD1306_SET_COM_OUT_DIR     _u(0xC0)
#define SSD1306_SET_COM_OUT_DIR_FLIP _u(0xC0)

#define SSD1306_SET_DISP_OFFSET     _u(0xD3)
#define SSD1306_SET_DISP_CLK_DIV    _u(0xD5)
#define SSD1306_SET_PRECHARGE       _u(0xD9)
#define SSD1306_SET_COM_PIN_CFG     _u(0xDA)
#define SSD1306_SET_VCOM_DESEL      _u(0xDB)

#define SSD1306_PAGE_HEIGHT         _u(8)
#define SSD1306_NUM_PAGES           (SSD1306_HEIGHT / SSD1306_PAGE_HEIGHT)
#define SSD1306_BUF_LEN             (SSD1306_NUM_PAGES * SSD1306_WIDTH)

#define SSD1306_WRITE_MODE         _u(0xFE)
#define SSD1306_READ_MODE          _u(0xFF)

// --------------------------------------------
// Used I2C #1, allows debugging with Picoprobe
#define PICO_SECOND_I2C 1
#define PICO_SECOND_I2C_SDA_PIN 6
#define PICO_SECOND_I2C_SCL_PIN 7

struct render_area {
    uint8_t start_col;
    uint8_t end_col;
    uint8_t start_page;
    uint8_t end_page;

    int buflen;
};

// Initialize render area for entire frame (SSD1306_WIDTH pixels by SSD1306_NUM_PAGES pages)
struct render_area frame_area = {
        start_col: 0,
        end_col : SSD1306_WIDTH - 1,
        start_page : 0,
        end_page : SSD1306_NUM_PAGES - 1
        };

uint8_t buf[SSD1306_BUF_LEN];

/**
 * NeoPixel LED Stuff
 * 
 */

#include "WS2812.hpp"

#define LED_PIN85 14    // Outer ring
#define LED_PIN65 15    // Inner ring
#define LED_LENGTH 24   // LEDs count

// Some constants for NeoPixel rings
#define RED_HIGH 64     // High red color intensity
#define RED_LOW1 1     // Low1 red color intensity
#define RED_LOW2 2     // Low2 red color intensity

#define BLUE_HIGH 64    // High blue color intensity
#define BLUE_LOW1 1     // Low1 blue color intensity
#define BLUE_LOW2 2     // Low2 blue color intensity
#define BLUE_LOW3 15    // Low3 blue color intensity
#define BLUE_LOW4 20    // Low4 blue color intensity

#define STRIP65_SHIFT 3 // Because Inner ring is mounted 3 LEDs shifted against Outer ring due mounting holes shift

// Forward declarations
void clear(WS2812 ledStrip);
void setDateTime(WS2812 ledStrip85, WS2812 ledStrip65, uint hours, uint minutes);
void test1(WS2812 ledStrip85, WS2812 ledStrip65);
void test2(WS2812 ledStrip85, WS2812 ledStrip65);
void test3(WS2812 ledStrip85, WS2812 ledStrip65);

//===========================================================================================
void calc_render_area_buflen(struct render_area *area) 
{
    // calculate how long the flattened buffer will be for a render area
    area->buflen = (area->end_col - area->start_col + 1) * (area->end_page - area->start_page + 1);
}

#ifdef i2c_default

void SSD1306_send_cmd(uint8_t cmd) 
{
    // I2C write process expects a control byte followed by data
    // this "data" can be a command or data to follow up a command
    // Co = 1, D/C = 0 => the driver expects a command
    uint8_t buf[2] = {0x80, cmd};
    i2c_write_blocking(/*i2c_default*/ i2c1, (SSD1306_I2C_ADDR & SSD1306_WRITE_MODE), buf, 2, false);
}

void SSD1306_send_cmd_list(uint8_t *buf, int num) 
{
    for (int i=0;i<num;i++)
        SSD1306_send_cmd(buf[i]);
}

void SSD1306_send_buf(uint8_t buf[], int buflen) 
{
    // in horizontal addressing mode, the column address pointer auto-increments
    // and then wraps around to the next page, so we can send the entire frame
    // buffer in one gooooooo!

    // copy our frame buffer into a new buffer because we need to add the control byte
    // to the beginning

    uint8_t *temp_buf = (uint8_t *) malloc(buflen + 1);

    temp_buf[0] = 0x40;
    memcpy(temp_buf+1, buf, buflen);

    i2c_write_blocking(i2c1, (SSD1306_I2C_ADDR & SSD1306_WRITE_MODE), temp_buf, buflen + 1, false);

    free(temp_buf);
}

void SSD1306_init() 
{
    // Some of these commands are not strictly necessary as the reset
    // process defaults to some of these but they are shown here
    // to demonstrate what the initialization sequence looks like
    // Some configuration values are recommended by the board manufacturer

    uint8_t cmds[] = {
        SSD1306_SET_DISP,               // set display off
        /* memory mapping */
        SSD1306_SET_MEM_MODE,           // set memory address mode 0 = horizontal, 1 = vertical, 2 = page
        0x00,                           // horizontal addressing mode
        /* resolution and layout */
        SSD1306_SET_DISP_START_LINE,    // set display start line to 0
        SSD1306_SET_SEG_REMAP | 0x01,   // set segment re-map, column address 127 is mapped to SEG0
        SSD1306_SET_MUX_RATIO,          // set multiplex ratio
        SSD1306_HEIGHT - 1,             // Display height - 1
        SSD1306_SET_COM_OUT_DIR | 0x08, // set COM (common) output scan direction. Scan from bottom up, COM[N-1] to COM0
        SSD1306_SET_DISP_OFFSET,        // set display offset
        0x00,                           // no offset
        SSD1306_SET_COM_PIN_CFG,        // set COM (common) pins hardware configuration. Board specific magic number. 
                                        // 0x02 Works for 128x32, 0x12 Possibly works for 128x64. Other options 0x22, 0x32
#if ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 32))
        0x02,                           
#elif ((SSD1306_WIDTH == 128) && (SSD1306_HEIGHT == 64))
        0x12,
#else
        0x02,
#endif
        /* timing and driving scheme */
        SSD1306_SET_DISP_CLK_DIV,       // set display clock divide ratio
        0x80,                           // div ratio of 1, standard freq
        SSD1306_SET_PRECHARGE,          // set pre-charge period
        0xF1,                           // Vcc internally generated on our board
        SSD1306_SET_VCOM_DESEL,         // set VCOMH deselect level
        0x30,                           // 0.83xVcc
        /* display */
        SSD1306_SET_CONTRAST,           // set contrast control
        0xFF,
        SSD1306_SET_ENTIRE_ON,          // set entire display on to follow RAM content
        SSD1306_SET_NORM_DISP,           // set normal (not inverted) display
        SSD1306_SET_CHARGE_PUMP,        // set charge pump
        0x14,                           // Vcc internally generated on our board
        SSD1306_SET_SCROLL | 0x00,      // deactivate horizontal scrolling if set. This is necessary as memory writes will corrupt if scrolling was enabled
        SSD1306_SET_DISP | 0x01, // turn display on
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

void SSD1306_scroll(bool on) 
{
    // configure horizontal scrolling
    uint8_t cmds[] = 
    {
        SSD1306_SET_HORIZ_SCROLL | 0x00,
        0x00, // dummy byte
        0x00, // start page 0
        0x00, // time interval
        0x03, // end page 3 SSD1306_NUM_PAGES ??
        0x00, // dummy byte
        0xFF//, // dummy byte
        //SSD1306_SET_SCROLL | (on ? 0x01 : 0) // Start/stop scrolling
    };

    SSD1306_send_cmd_list(cmds, count_of(cmds));
}

void render(uint8_t *buf, struct render_area *area) 
{
    // update a portion of the display with a render area
    uint8_t cmds[] = {
        SSD1306_SET_COL_ADDR,
        area->start_col,
        area->end_col,
        SSD1306_SET_PAGE_ADDR,
        area->start_page,
        area->end_page
    };
    
    SSD1306_send_cmd_list(cmds, count_of(cmds));
    SSD1306_send_buf(buf, area->buflen);
}

static void SetPixel(uint8_t *buf, int x,int y, bool on) 
{
    assert(x >= 0 && x < SSD1306_WIDTH && y >=0 && y < SSD1306_HEIGHT);

    // The calculation to determine the correct bit to set depends on which address
    // mode we are in. This code assumes horizontal

    // The video ram on the SSD1306 is split up in to 8 rows, one bit per pixel.
    // Each row is 128 long by 8 pixels high, each byte vertically arranged, so byte 0 is x=0, y=0->7,
    // byte 1 is x = 1, y=0->7 etc

    // This code could be optimised, but is like this for clarity. The compiler
    // should do a half decent job optimising it anyway.

    const int BytesPerRow = SSD1306_WIDTH ; // x pixels, 1bpp, but each row is 8 pixel high, so (x / 8) * 8

    int byte_idx = (y / 8) * BytesPerRow + x;
    uint8_t byte = buf[byte_idx];

    if (on)
        byte |=  1 << (y % 8);
    else
        byte &= ~(1 << (y % 8));

    buf[byte_idx] = byte;
}

// Basic Bresenhams.
/*static void DrawLine(uint8_t *buf, int x0, int y0, int x1, int y1, bool on) 
{
    int dx =  abs(x1-x0);
    int sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0);
    int sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    int e2;

    while (true) {
        SetPixel(buf, x0, y0, on);
        if (x0 == x1 && y0 == y1)
            break;
        e2 = 2*err;

        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}*/

static inline int GetFontIndex(uint8_t ch) 
{
    if (ch >= 'A' && ch <='Z') {
        return  ch - 'A' + 1;
    }
    else if (ch >= '0' && ch <='9') {
        return  ch - '0' + 27;
    }
    else return  0; // Not got that char so space.
}

static uint8_t reversed[sizeof(font)] = {0};

static uint8_t reverse(uint8_t b) 
{
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}
static void FillReversedCache() 
{
    // calculate and cache a reversed version of fhe font, because I defined it upside down...doh!
    for (int i=0;i<sizeof(font);i++)
        reversed[i] = reverse(font[i]);
}

static void WriteChar(uint8_t *buf, int16_t x, int16_t y, uint8_t ch) 
{
    if (reversed[0] == 0) 
        FillReversedCache();

    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    // For the moment, only write on Y row boundaries (every 8 vertical pixels)
    y = y/8;

    ch = toupper(ch);
    int idx = GetFontIndex(ch);
    int fb_idx = y * 128 + x;

    for (int i=0;i<8;i++) 
    {
        buf[fb_idx++] = reversed[idx * 8 + i];
    }
}

static void WriteString(uint8_t *buf, int16_t x, int16_t y, const char *str)
{
    // Cull out any string off the screen
    if (x > SSD1306_WIDTH - 8 || y > SSD1306_HEIGHT - 8)
        return;

    while (*str)
    {
        WriteChar(buf, x, y, *str++);
        x+=8;
    }
}

void displayTime(struct tm *utc, WS2812 ledStrip85, WS2812 ledStrip65)
{
    // zero the entire display
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    char buffer[20];
    int y = 0;

        //----------------------------------------------------------------------------------------
        // UTC Date
        sprintf(buffer, "Date Time UTC");
        WriteString(buf, 5, y, buffer);
        y+=8;

        sprintf(buffer, "%02d/%02d/%04d", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900);
        WriteString(buf, 5, y, buffer);
        y+=8;
        
        //----------------------------------------------------------------------------------------
        // UTC Time
        sprintf(buffer, "%02d:%02d:%02d", utc->tm_hour, utc->tm_min, utc->tm_sec);
        WriteString(buf, 5, y, buffer);
        y+=8;

        sprintf(buffer, "          ");
        WriteString(buf, 5, y, buffer);
        y+=8;
        
        //----------------------------------------------------------------------------------------
        // Convert UTC to Local time. Adjust this code with regards of Your location.

        uint maxDayInAnyMonth[13] = { 0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
        if(utc->tm_year %4 == 0) { maxDayInAnyMonth[2] = 29; } //adjust for leapyear
        uint maxDayUtcMth = maxDayInAnyMonth[utc->tm_mon];
        uint maxDayPrevMth = maxDayInAnyMonth[utc->tm_mon-1];
        if(!maxDayPrevMth) { maxDayPrevMth = 31; } //month before utc month
        
        int localHour = utc->tm_hour;
        int localDay = utc->tm_mday;
        int localMonth = utc->tm_mon + 1;
        int localYear = utc->tm_year;

        if (++localHour > 23) // UTC+1 = CET
        {
            localHour = 0;

            if (++localDay > maxDayInAnyMonth[maxDayUtcMth])
            {
                localDay = 1;

                if (++localMonth > 12)
                {
                    localMonth = 1;
                    localYear++;
                }
            }
        }

        bool isDST = gpio_get(BUTTON_DST);

        if (isDST)
        {
            if (++localHour > 23) // UTC+2 = CEST
            {
                localHour = 0;

                if (++localDay > maxDayInAnyMonth[maxDayUtcMth])
                {
                    localDay = 1;

                    if (++localMonth > 12)
                    {
                        localMonth = 1;
                        localYear++;
                    }
                }
            }
        }
        
        //----------------------------------------------------------------------------------------
        // LocalDate
        if (isDST)
        {
            sprintf(buffer, "Date Time CEST");
        }
        else
        {
            sprintf(buffer, "Date Time CET");
        } 
        WriteString(buf, 5, y, buffer);
        y+=8;

        sprintf(buffer, "%02d/%02d/%04d", localDay, localMonth, localYear + 1900);
        WriteString(buf, 5, y, buffer);
        y+=8;

        //----------------------------------------------------------------------------------------
        // Local Time      
        
        sprintf(buffer, "%02d:%02d:%02d", localHour, utc->tm_min, utc->tm_sec);
        WriteString(buf, 5, y, buffer);
        y+=8;

        sprintf(buffer, "          ");
        WriteString(buf, 5, y, buffer);

        render(buf, &frame_area);

    //----------------------------------------------------------------------------------------
    // Neopixels
    setDateTime(ledStrip85, ledStrip65, localHour, utc->tm_min);
}

#endif

/**
 * NTP stuff, code borrowed from pico_examples/pico_w/ntp_client
 * 
 */

// Called with results of operation
static void ntp_result(NTP_T* state, int status, time_t *result) 
{
    if (status == 0 && result) 
    {
        struct tm *utc = gmtime(result);
        printf("got ntp response: %02d/%02d/%04d %02d:%02d:%02d\n", utc->tm_mday, utc->tm_mon + 1, utc->tm_year + 1900,
               utc->tm_hour, utc->tm_min, utc->tm_sec);
    }

    if (state->ntp_resend_alarm > 0) 
    {
        cancel_alarm(state->ntp_resend_alarm);
        state->ntp_resend_alarm = 0;
    }
    
    state->ntp_test_time = make_timeout_time_ms(NTP_TEST_TIME);
    state->dns_request_sent = false;
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data);

// Make an NTP request
static void ntp_request(NTP_T *state) 
{
    // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
    // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
    // these calls are a no-op and can be omitted, but it is a good practice to use them in
    // case you switch the cyw43_arch type later.
    cyw43_arch_lwip_begin();
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, NTP_MSG_LEN, PBUF_RAM);
    uint8_t *req = (uint8_t *) p->payload;
    memset(req, 0, NTP_MSG_LEN);
    req[0] = 0x1b;
    udp_sendto(state->ntp_pcb, p, &state->ntp_server_address, NTP_PORT);
    pbuf_free(p);
    cyw43_arch_lwip_end();
}

static int64_t ntp_failed_handler(alarm_id_t id, void *user_data)
{
    NTP_T* state = (NTP_T*)user_data;
    printf("ntp request failed\n");
    ntp_result(state, -1, NULL);
    return 0;
}

// Call back with a DNS result
static void ntp_dns_found(const char *hostname, const ip_addr_t *ipaddr, void *arg) {
    NTP_T *state = (NTP_T*)arg;
    if (ipaddr) 
    {
        state->ntp_server_address = *ipaddr;
        printf("ntp address %s\n", ipaddr_ntoa(ipaddr));
        ntp_request(state);
    } 
    else 
    {
        printf("ntp dns request failed\n");
        ntp_result(state, -1, NULL);
    }
}

// NTP data received
static void ntp_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) 
{
    NTP_T *state = (NTP_T*)arg;
    uint8_t mode = pbuf_get_at(p, 0) & 0x7;
    uint8_t stratum = pbuf_get_at(p, 1);

    // Check the result
    if (ip_addr_cmp(addr, &state->ntp_server_address) && port == NTP_PORT && p->tot_len == NTP_MSG_LEN && mode == 0x4 && stratum != 0) 
    {
        uint8_t seconds_buf[4] = {0};
        pbuf_copy_partial(p, seconds_buf, sizeof(seconds_buf), 40);
        uint32_t seconds_since_1900 = seconds_buf[0] << 24 | seconds_buf[1] << 16 | seconds_buf[2] << 8 | seconds_buf[3];
        uint32_t seconds_since_1970 = seconds_since_1900 - NTP_DELTA;
        
        // Global var.
        epoch = seconds_since_1970;
        ntp_result(state, 0, &epoch);
    } 
    else 
    {
        printf("invalid ntp response\n");
        ntp_result(state, -1, NULL);
    }
    pbuf_free(p);
}

// Perform initialisation
static NTP_T* ntp_init(void) 
{
    NTP_T *state = (NTP_T*) calloc(1, sizeof(NTP_T));
    if (!state) 
    {
        printf("failed to allocate state\n");
        return NULL;
    }
    
    state->ntp_pcb = udp_new_ip_type(IPADDR_TYPE_ANY);
    if (!state->ntp_pcb) 
    {
        printf("failed to create pcb\n");
        free(state);
        return NULL;
    }

    udp_recv(state->ntp_pcb, ntp_recv, state);
    return state;
}

// Runs ntp test forever
void run_ntp_test(NTP_T *state) 
{
    if (!state)
        return;

    while(true) 
    {
        if (absolute_time_diff_us(get_absolute_time(), state->ntp_test_time) < 0 && !state->dns_request_sent) 
        {
            // Set alarm in case udp requests are lost
            state->ntp_resend_alarm = add_alarm_in_ms(NTP_RESEND_TIME, ntp_failed_handler, state, true);

            // cyw43_arch_lwip_begin/end should be used around calls into lwIP to ensure correct locking.
            // You can omit them if you are in a callback from lwIP. Note that when using pico_cyw_arch_poll
            // these calls are a no-op and can be omitted, but it is a good practice to use them in
            // case you switch the cyw43_arch type later.
            cyw43_arch_lwip_begin();
            int err = dns_gethostbyname(NTP_SERVER, &state->ntp_server_address, ntp_dns_found, state);
            cyw43_arch_lwip_end();

            state->dns_request_sent = true;
            if (err == ERR_OK) 
            {
                ntp_request(state); // Cached result
                return;
            } 
            else if (err != ERR_INPROGRESS) 
            {   // ERR_INPROGRESS means expect a callback
                printf("dns request failed\n");
                ntp_result(state, -1, NULL);
            }
        }
#if PICO_CYW43_ARCH_POLL 
        // if you are using pico_cyw43_arch_poll, then you must poll periodically from your
        // main loop (not from a timer interrupt) to check for Wi-Fi driver or lwIP work that needs to be done.
        cyw43_arch_poll();
        // you can poll as often as you like, however if you have nothing else to do you can
        // choose to sleep until either a specified time, or cyw43_arch_poll() has work to do:
        cyw43_arch_wait_for_work_until(state->dns_request_sent ? at_the_end_of_time : state->ntp_test_time);
#else
        // if you are not using pico_cyw43_arch_poll, then WiFI driver and lwIP work
        // is done via interrupt in the background. This sleep is just an example of some (blocking)
        // work you might be doing.
        sleep_ms(500);
#endif
    }
}

/**
 * Clear given strip 
 */
void clear(WS2812 ledStrip)
{
    ledStrip.fill(0, 0, LED_LENGTH);
    ledStrip.show();
}

    
void setDateTime(WS2812 ledStrip85, WS2812 ledStrip65, uint hours, uint minutes)
{
    if (hours >= 12)
    {
        hours -= 12;
    }

    critical_section_enter_blocking (&myLock);

    clear(ledStrip85);
    clear(ledStrip65);

    ledStrip85.fill(0, 0, LED_LENGTH);
    ledStrip65.fill(0, 0, LED_LENGTH);

    uint minutesShift = 0;
    if (minutes > 30)
    {
        minutesShift = 1;
    }

    // Green ticks
    ledStrip85.setPixelColor(18, 0, 20, 0);
    ledStrip85.setPixelColor(0, 0, 20, 0);
    ledStrip85.setPixelColor(6, 0, 20, 0);
    ledStrip85.setPixelColor(12, 0, 20, 0);

    ledStrip65.setPixelColor(15, 0, 20, 0);
    ledStrip65.setPixelColor(21, 0, 20, 0);
    ledStrip65.setPixelColor(3, 0, 20, 0);
    ledStrip65.setPixelColor(9, 0, 20, 0);

    int index65;

    switch (hours)
    {
        case 0:
            index65 = 18 + minutesShift - STRIP65_SHIFT;
            break;

        case 1:
            index65 = 20 + minutesShift - STRIP65_SHIFT;
            break;

        case 2:
            index65 = 22 + minutesShift - STRIP65_SHIFT, RED_HIGH;
            break;

        case 3:
            index65 = 24 + minutesShift - STRIP65_SHIFT;
            break;

        case 4:
            index65 = 26 + minutesShift - STRIP65_SHIFT;
            if (index65 > 23) { index65 = 0; }
            break;

        case 5:
            index65 = 4 + minutesShift - STRIP65_SHIFT;
            break;

        case 6:
            index65 = 6 + minutesShift - STRIP65_SHIFT;
            break;

        case 7:
            index65 = 8 + minutesShift - STRIP65_SHIFT;
            break;

        case 8:
            index65 = 10 + minutesShift - STRIP65_SHIFT;
            break;

        case 9:
            index65 = 12 + minutesShift - STRIP65_SHIFT;
            break;

        case 10:
            index65 = 14 + minutesShift - STRIP65_SHIFT;
            break;

        case 11:
            index65 = 16 + minutesShift - STRIP65_SHIFT;
            break;
    }

    ledStrip65.setPixelColor(index65, RED_HIGH, 0, 0);
    printf("ledStrip65: %02d RED_HIGH", index65);

    if ((minutes > 1) && (minutes < 15))
    {
        ledStrip65.setPixelColor(index65+1, RED_LOW1, 0, 0);
        printf("ledStrip65: %02d RED_LOW1", index65+1);
    }
    else if ((minutes > 15) && (minutes < 30))
    {
        ledStrip65.setPixelColor(index65+1, RED_LOW2, 0, 0);
        printf("ledStrip65: %02d RED_LOW2", index65);
    }
    else if ((minutes > 30) && (minutes < 45))
    {
        ledStrip65.setPixelColor(index65+1, RED_LOW1, 0, 0);
        printf("ledStrip65: %02d RED_LOW1", index65);
    }
    else if ((minutes > 45) && (minutes <= 59))
    {
        ledStrip65.setPixelColor(index65+1, RED_LOW2, 0, 0);
        printf("ledStrip65: %02d RED_LOW2", index65);
    }

    switch (minutes)
    {
        case 0:
            ledStrip85.setPixelColor(18, 0, 0, BLUE_HIGH);
            break;

        case 1:
            ledStrip85.setPixelColor(18, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(19, 0, 0, BLUE_LOW1);
            break;

        case 2:
            ledStrip85.setPixelColor(18, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(19, 0, 0, BLUE_LOW2);
            break;

        case 3:
            ledStrip85.setPixelColor(18, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(19, 0, 0, BLUE_LOW3);
            break;

        case 4:
            ledStrip85.setPixelColor(18, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(19, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        
        case 5:
            ledStrip85.setPixelColor(20, 0, 0, BLUE_HIGH);
            break;

        case 6:
            ledStrip85.setPixelColor(20, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(21, 0, 0, BLUE_LOW1);
            break;

        case 7:
            ledStrip85.setPixelColor(20, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(21, 0, 0, BLUE_LOW2);
            break;

        case 8:
            ledStrip85.setPixelColor(20, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(21, 0, 0, BLUE_LOW3);
            break;

        case 9:
            ledStrip85.setPixelColor(20, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(21, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------

        case 10:
            ledStrip85.setPixelColor(22, 0, 0, BLUE_HIGH);
            break;

        case 11:
            ledStrip85.setPixelColor(22, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(23, 0, 0, BLUE_LOW1);
            break;

        case 12:
            ledStrip85.setPixelColor(22, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(23, 0, 0, BLUE_LOW2);
            break;

        case 13:
            ledStrip85.setPixelColor(22, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(23, 0, 0, BLUE_LOW3);
            break;

        case 14:
            ledStrip85.setPixelColor(22, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(23, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------

        case 15:
            ledStrip85.setPixelColor(0, 0, 0, BLUE_HIGH);
            break;

        case 16:
            ledStrip85.setPixelColor(0, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(1, 0, 0, BLUE_LOW1);
            break;

        case 17:
            ledStrip85.setPixelColor(0, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(1, 0, 0, BLUE_LOW2);
            break;

        case 18:
            ledStrip85.setPixelColor(0, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(1, 0, 0, BLUE_LOW3);
            break;

        case 19:
            ledStrip85.setPixelColor(0, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(1, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        
        case 20:
            ledStrip85.setPixelColor(2, 0, 0, BLUE_HIGH);
            break;

        case 21:
            ledStrip85.setPixelColor(2, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(3, 0, 0, BLUE_LOW1);
            break;

        case 22:
            ledStrip85.setPixelColor(2, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(3, 0, 0, BLUE_LOW2);
            break;

        case 23:
            ledStrip85.setPixelColor(2, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(3, 0, 0, BLUE_LOW3);
            break;

        case 24:
            ledStrip85.setPixelColor(2, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(3, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------

        case 25:
            ledStrip85.setPixelColor(4, 0, 0, BLUE_HIGH);
            break;

        case 26:
            ledStrip85.setPixelColor(4, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(5, 0, 0, BLUE_LOW1);
            break;

        case 27:
            ledStrip85.setPixelColor(4, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(5, 0, 0, BLUE_LOW2);
            break;

        case 28:
            ledStrip85.setPixelColor(4, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(5, 0, 0, BLUE_LOW3);
            break;

        case 29:
            ledStrip85.setPixelColor(4, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(5, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------

        case 30:
            ledStrip85.setPixelColor(6, 0, 0, BLUE_HIGH);
            break;

        case 31:
            ledStrip85.setPixelColor(6, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(7, 0, 0, BLUE_LOW1);
            break;

        case 32:
            ledStrip85.setPixelColor(6, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(7, 0, 0, BLUE_LOW2);
            break;

        case 33:
            ledStrip85.setPixelColor(6, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(7, 0, 0, BLUE_LOW3);
            break;

        case 34:
            ledStrip85.setPixelColor(6, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(7, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        
        case 35:
            ledStrip85.setPixelColor(8, 0, 0, BLUE_HIGH);
            break;

        case 36:
            ledStrip85.setPixelColor(8, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(9, 0, 0, BLUE_LOW1);
            break;

        case 37:
            ledStrip85.setPixelColor(8, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(9, 0, 0, BLUE_LOW2);
            break;

        case 38:
            ledStrip85.setPixelColor(8, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(9, 0, 0, BLUE_LOW3);
            break;

        case 39:
            ledStrip85.setPixelColor(8, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(9, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------

        case 40:
            ledStrip85.setPixelColor(10, 0, 0, BLUE_HIGH);
            break;

        case 41:
            ledStrip85.setPixelColor(10, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(11, 0, 0, BLUE_LOW1);
            break;

        case 42:
            ledStrip85.setPixelColor(10, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(11, 0, 0, BLUE_LOW2);
            break;

        case 43:
            ledStrip85.setPixelColor(10, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(11, 0, 0, BLUE_LOW3);
            break;

        case 44:
            ledStrip85.setPixelColor(10, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(11, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        //-------------------------------------------------------------------------------------

        case 45:
            ledStrip85.setPixelColor(12, 0, 0, BLUE_HIGH);
            break;

        case 46:
            ledStrip85.setPixelColor(12, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(13, 0, 0, BLUE_LOW1);
            break;

        case 47:
            ledStrip85.setPixelColor(12, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(13, 0, 0, BLUE_LOW2);
            break;

        case 48:
            ledStrip85.setPixelColor(12, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(13, 0, 0, BLUE_LOW3);
            break;

        case 49:
            ledStrip85.setPixelColor(12, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(13, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------
        
        case 50:
            ledStrip85.setPixelColor(14, 0, 0, BLUE_HIGH);
            break;

        case 51:
            ledStrip85.setPixelColor(14, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(15, 0, 0, BLUE_LOW1);
            break;

        case 52:
            ledStrip85.setPixelColor(14, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(15, 0, 0, BLUE_LOW2);
            break;

        case 53:
            ledStrip85.setPixelColor(14, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(15, 0, 0, BLUE_LOW3);
            break;

        case 54:
            ledStrip85.setPixelColor(14, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(15, 0, 0, BLUE_LOW4);
            break;            

        //-------------------------------------------------------------------------------------

        case 55:
            ledStrip85.setPixelColor(16, 0, 0, BLUE_HIGH);
            break;

        case 56:
            ledStrip85.setPixelColor(16, 0, 0, BLUE_LOW4);
            ledStrip85.setPixelColor(17, 0, 0, BLUE_LOW1);
            break;

        case 57:
            ledStrip85.setPixelColor(16, 0, 0, BLUE_LOW3);
            ledStrip85.setPixelColor(17, 0, 0, BLUE_LOW2);
            break;

        case 58:
            ledStrip85.setPixelColor(16, 0, 0, BLUE_LOW2);
            ledStrip85.setPixelColor(17, 0, 0, BLUE_LOW3);
            break;

        case 59:
            ledStrip85.setPixelColor(16, 0, 0, BLUE_LOW1);
            ledStrip85.setPixelColor(17, 0, 0, BLUE_LOW4);
            break;            
    }

    ledStrip85.show();
    ledStrip65.show();

    critical_section_exit(&myLock);
}

void test1(WS2812 ledStrip85, WS2812 ledStrip65)
{
    // 1. Set all LEDs to red!
    printf("1. Set all LEDs to red!");
    ledStrip85.fill( WS2812::RGB(255, 0, 0) );
    ledStrip85.show();
    ledStrip65.fill( WS2812::RGB(255, 0, 0) );
    ledStrip65.show();
    sleep_ms(1000);

    // 2. Set all LEDs to green!
    printf("2. Set all LEDs to green!");
    ledStrip85.fill( WS2812::RGB(0, 255, 0) );
    ledStrip85.show();
    ledStrip65.fill( WS2812::RGB(0, 255, 0) );
    ledStrip65.show();
    sleep_ms(1000);

    // 3. Set all LEDs to blue!
    printf("3. Set all LEDs to blue!");
    ledStrip85.fill( WS2812::RGB(0, 0, 255) );
    ledStrip85.show();
    ledStrip65.fill( WS2812::RGB(0, 0, 255) );
    ledStrip65.show();
    sleep_ms(1000);

    // 4. Set half LEDs to red and half to blue!
    printf("4. Set half LEDs to red and half to blue!");
    ledStrip85.fill(WS2812::RGB(255, 0, 0), 0, LED_LENGTH / 2 );
    ledStrip85.fill(WS2812::RGB(0, 0, 255), LED_LENGTH / 2 );
    ledStrip65.fill(WS2812::RGB(0, 0, 255), 0, LED_LENGTH / 2 );
    ledStrip65.fill(WS2812::RGB(255, 0, 0), LED_LENGTH / 2 );
    ledStrip85.show();
    sleep_ms(1000);

    // 5. Do some fancy animation
    printf("5. Do some fancy animation");
    
    for (int i = 0; i < 10; i++)
    {
        // Pick a random color
        uint32_t color85 = (uint32_t) rand();
        uint32_t color65 = (uint32_t) rand();

        // Pick a random direction
        int8_t dir = (rand() & 1 ? 1 : -1);

        // Setup start and end offsets for the loop
        uint8_t start = (dir > 0 ? 0 : LED_LENGTH);
        uint8_t end = (dir > 0 ? LED_LENGTH : 0);

        for (uint8_t ledIndex = start; ledIndex != end; ledIndex += dir) 
        {
            ledStrip85.setPixelColor(ledIndex, color85);
            ledStrip85.show();
            ledStrip65.setPixelColor(ledIndex, color65);
            ledStrip65.show();
            sleep_ms(50);
        }
    }
}

void test2(WS2812 ledStrip85, WS2812 ledStrip65)
{
    // zero the entire display
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    char buffer[20];

    for (uint hours = 0; hours < 12; hours++)
    {
        sprintf(buffer, "%02d:%02d:%02d", hours, 0, 0);
        WriteString(buf, 5, 20, buffer);
        render(buf, &frame_area);
            
        setDateTime(ledStrip85, ledStrip65, hours, 0);

        sleep_ms(500);
    }
}

void test3(WS2812 ledStrip85, WS2812 ledStrip65)
{
    // zero the entire display
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    char buffer[20];

    for (uint hours = 0; hours < 12; hours++)
    {
        for (uint minutes = 0; minutes < 60; minutes++)
        {
            sprintf(buffer, "%02d:%02d:%02d", hours, minutes, 0);
            WriteString(buf, 5, 20, buffer);
            render(buf, &frame_area);
            
            setDateTime(ledStrip85, ledStrip65, hours, minutes);

            sleep_ms(50);
        }
        sleep_ms(100);
    }
}

// WiFi config from SD card. If mot readed, config from CMakeList.txt is used
char wifiSSID[] = "                     ";
char wifiPwd[] = "                     ";

/**
 * Read WiFi config from SD Card
 * 
 * Used library:
 * https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico/tree/master
 */
bool readWiFiConfig()
{
    FRESULT fr;
    FATFS fs;
    FIL fil;
    const char filename[] = "wifi.txt";

    // Initialize SD card
    if (!sd_init_driver()) 
    {
        printf("ERROR: Could not initialize SD card\r\n");
        return false;
    }

    // Mount drive
    fr = f_mount(&fs, "0:", 1);
    if (fr != FR_OK) 
    {
        printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
        return false;
    }

    // Open file for reading
    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) 
    {
        printf("ERROR: Could not open file (%d)\r\n", fr);
        return false;
    }

    // Print every line in file over serial
    printf("Reading from file '%s':\r\n", filename);
    
    f_gets(wifiSSID, 20, &fil);
    printf(wifiSSID);
    for (int i = 0; i < 20; i++)
    {
        if ((wifiSSID[i] == '\r') || (wifiSSID[i] == '\n'))
        {
            wifiSSID[i] = 0;
            break;
        }
    }

    f_gets(wifiPwd, 20, &fil);
    printf(wifiPwd);
    for (int i = 0; i < 20; i++)
    {
        if ((wifiPwd[i] == '\r') || (wifiPwd[i] == '\n'))
        {
            wifiPwd[i] = 0;
            break;
        }
    }
    
    printf("\r\n");

    // Close file
    f_close(&fil);

    // Unmount drive
    f_unmount("0:");

    return true;
}

/**
 * Common main function for combining NTP over WiFi, NeoPixel and OLED displays
 * \\192.168.9.102\SHARE
 */
int main() 
{
    stdio_init_all();
    critical_section_init(&myLock);

    gpio_init(BUTTON_DST);
    gpio_set_dir(BUTTON_DST, GPIO_IN);

    bool wifiConfig = readWiFiConfig();

    if (cyw43_arch_init()) 
    {
        printf("failed to initialise\n");
        sleep_ms(3000);
        return 0;
    }

    cyw43_arch_enable_sta_mode();

    int result;

    if (wifiConfig)
    {
        result = cyw43_arch_wifi_connect_timeout_ms(wifiSSID, wifiPwd, CYW43_AUTH_WPA2_AES_PSK, 10000);
    }
    else
    {
         result = cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000);
    }

    if (result) 
    {
        printf("Failed to connect. Error code: %d\n", result);
        printf(WIFI_SSID);
        printf(WIFI_PASSWORD);
        sleep_ms(3000);
        return 0;
    }

    //----------------------------------------------------------------------------------------

    //=========================================================================================================

#if !defined(i2c_default) || !defined(PICO_SECOND_I2C_SDA_PIN) || !defined(PICO_SECOND_I2C_SCL_PIN)
#warning i2c / SSD1306_i2d example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    // useful information for picotool
    bi_decl(bi_2pins_with_func(PICO_SECOND_I2C_SDA_PIN, PICO_SECOND_I2C_SCL_PIN, GPIO_FUNC_I2C));
    bi_decl(bi_program_description("SSD1306 OLED driver I2C example for the Raspberry Pi Pico"));

    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(/*i2c_default*/ i2c1, SSD1306_I2C_CLK * 1000);
    gpio_set_function(PICO_SECOND_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_SECOND_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_SECOND_I2C_SDA_PIN);
    gpio_pull_up(PICO_SECOND_I2C_SCL_PIN);

    // run through the complete initialization process
    SSD1306_init();

    calc_render_area_buflen(&frame_area);

    // zero the entire display
    memset(buf, 0, SSD1306_BUF_LEN);
    render(buf, &frame_area);

    // intro sequence: flash the screen 3 times
    for (int i = 0; i < 3; i++) {
        SSD1306_send_cmd(SSD1306_SET_ALL_ON);    // Set all pixels on
        sleep_ms(500);
        SSD1306_send_cmd(SSD1306_SET_ENTIRE_ON); // go back to following RAM for pixel state
        sleep_ms(500);
    }

    const char *text[] = {
        "Raspberry PI",
        "PICO W OLED ",
        "and Neopixel",
        "NTP clock",
        "               ",
        "Wait for NTP",
        "sychronization",
        "And Enjoy"
    };

    int y = 0;
    for (int i = 0 ; i < count_of(text); i++) 
    {
        WriteString(buf, 5, y, text[i]);
        y+=8;
    }

    render(buf, &frame_area);
    sleep_ms(100);

    //----------------------------------------------------------------------------------------
    // I found this (unconfirmed) info:
    //
    // The CYW43 driver gets installed in PIO1 by default, and only uses PIO0 if there's no space.
    //
    // So I hope that using PIO0 SM2 and SM3 avoid collision

    if (pio_sm_is_claimed(pio0, 2))
    {
        printf("PIO0 SM2 is claimed\n");
    }

WS2812 ledStrip85(
        LED_PIN85,          // Data line (GP14)
        LED_LENGTH,         // Strip is 24 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        2,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip was: FORMAT_GRB FORMAT_RGB
    );

    if (pio_sm_is_claimed(pio0, 3))
    {
        printf("PIO0 SM3 is claimed\n");
    }

WS2812 ledStrip65(
        LED_PIN65,          // Data line (GP15)
        LED_LENGTH,         // Strip is 24 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        3,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip was: FORMAT_GRB FORMAT_RGB
    );

    test2(ledStrip85, ledStrip65);
    clear(ledStrip85);
    clear(ledStrip65);

    test3(ledStrip85, ledStrip65);
    clear(ledStrip85);
    clear(ledStrip65);
    
    //test1(ledStrip85, ledStrip65);
    //clear(ledStrip85);
    //clear(ledStrip65);

    //=========================================================================================================

    NTP_T *state = ntp_init();

    while (true)
    {
        //----------------------------------------------------------------------------------------
        run_ntp_test(state);
        
        struct tm *utc = gmtime(&epoch);
        
        displayTime(utc, ledStrip85, ledStrip65);

        sleep_ms(3000);
    }

    free(state);

#endif

    //=========================================================================================================
    cyw43_arch_deinit();
    critical_section_deinit(&myLock);

    return 0;
}
