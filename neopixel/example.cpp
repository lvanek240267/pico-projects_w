
/*
 * NeoPixel analog clocks
 * 
 * NeoPixel library:
 * https://github.com/ForsakenNGS/Pico_WS2812
 * 
 * Copyleft Lumir Vanek, vanek.lumir@gmail.com
 */

#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "WS2812.hpp"

#define LED_PIN85 14    // Outer ring
#define LED_PIN65 15    // Inner ring
#define LED_LENGTH 24   // LEDs count

// Some constants for NeoPixel rings
#define RED_HIGH 64     // High red color intensity
#define RED_LOW 10      // Low red color intensity
#define BLUE_HIGH 64    // High blue color intensity
#define BLUE_LOW1 3     // Low1 blue color intensity
#define BLUE_LOW2 8     // Low2 blue color intensity
#define BLUE_LOW3 15    // Low3 blue color intensity
#define BLUE_LOW4 20    // Low4 blue color intensity
#define STRIP65_SHIFT 2 // Because Inner ring is mounted 2 LEDs shifted against Outer ring due mounting holes shift

// Forward declarations
void clear(WS2812 ledStrip);
void setHours(WS2812 ledStrip85, WS2812 ledStrip65, uint hours, uint minutes);
void test2(WS2812 ledStrip85, WS2812 ledStrip65);


int main()
{
    stdio_init_all();

    // 0. Initialize LED strips
    printf("0. Initialize LED strips");

    WS2812 ledStrip85(
        LED_PIN85,          // Data line is connected to pin 0. (GP14)
        LED_LENGTH,         // Strip is 24 LEDs long.
        pio0,               // Use PIO 0 for creating the state machine.
        0,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip was: FORMAT_GRB FORMAT_RGB
    );

    WS2812 ledStrip65(
        LED_PIN65,          // Data line is connected to pin 0. (GP15)
        LED_LENGTH,         // Strip is 24 LEDs long.
        pio1,               // Use PIO 1 for creating the state machine.
        0,                  // Index of the state machine that will be created for controlling the LED strip
                            // You can have 4 state machines per PIO-Block up to 8 overall.
                            // See Chapter 3 in: https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
        WS2812::FORMAT_GRB  // Pixel format used by the LED strip was: FORMAT_GRB FORMAT_RGB
    );

    test2(ledStrip85, ledStrip65);

    //--------------------------------------------------------------------------------------

    clear(ledStrip85);
    clear(ledStrip65);

    //--------------------------------------------------------------------------------------

    /*ledStrip85.setPixelColor(0, 64, 0, 0);
    ledStrip65.setPixelColor(LED_LENGTH - 2, 0, 0, 64);
    ledStrip85.show();
    ledStrip65.show();*/

    /*
     * Test clock setting
     */
    for (uint hours = 0; hours <= 12; hours++)
    {
        for (uint minutes = 0; minutes < 60; minutes++)
        {
            setHours(ledStrip85, ledStrip65, hours, minutes);
            sleep_ms(50);
        }
    }

    while (true)
    {
        for (uint hours = 0; hours <= 12; hours++)
        {
            for (uint minutes = 0; minutes < 60; minutes++)
            {
                setHours(ledStrip85, ledStrip65, hours, minutes);
                sleep_ms(300);
            }
        }
    }

    return 0;
}

/**
 * Clear given strip 
 */
void clear(WS2812 ledStrip)
{
    ledStrip.fill(0, 0, LED_LENGTH);
    ledStrip.show();
}



void setHours(WS2812 ledStrip85, WS2812 ledStrip65, uint hours, uint minutes)
{
    if (hours >= 12)
    {
        hours -= 12;
    }

    clear(ledStrip85);
    clear(ledStrip65);

    uint minutesShift = 0;
    if (minutes > 30)
    {
        minutesShift = 1;
    }
    
    switch (hours)
    {
        case 0:
            //ledStrip85.setPixelColor(18 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(18 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 1:
            //ledStrip85.setPixelColor(20 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(20 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 2:
            //ledStrip85.setPixelColor(22 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(22 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 3:
            //ledStrip85.setPixelColor(0 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(22 + minutesShift, RED_HIGH, 0, 0);
            break;

        case 4:
            //ledStrip85.setPixelColor(2 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(2 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 5:
            //ledStrip85.setPixelColor(4 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(4 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 6:
            //ledStrip85.setPixelColor(6 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(6 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 7:
            //ledStrip85.setPixelColor(8 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(8 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 8:
            //ledStrip85.setPixelColor(10 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(10 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 9:
            //ledStrip85.setPixelColor(12 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(12 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 10:
            //ledStrip85.setPixelColor(14 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(14 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;

        case 11:
            //ledStrip85.setPixelColor(16 + minutesShift, RED_HIGH, 0, 0);
            ledStrip65.setPixelColor(16 + minutesShift - STRIP65_SHIFT, RED_HIGH, 0, 0);
            break;
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
}

void test2(WS2812 ledStrip85, WS2812 ledStrip65)
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
    ledStrip85.fill( WS2812::RGB(255, 0, 0), 0, LED_LENGTH / 2 );
    ledStrip85.fill( WS2812::RGB(0, 0, 255), LED_LENGTH / 2 );
    ledStrip65.fill( WS2812::RGB(0, 0, 255), 0, LED_LENGTH / 2 );
    ledStrip65.fill( WS2812::RGB(255, 0, 0), LED_LENGTH / 2 );
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
