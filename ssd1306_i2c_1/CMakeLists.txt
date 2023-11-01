project(picow_ntp_client_background C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)


add_executable(picow_ntp_client_background
        ssd1306_i2c_1.cpp
        WS2812.cpp
        sd_card.c
        ff.c
        ffsystem.c
        ffunicode.c
        spi.c
        sd_spi.c
        crc.c
        my_debug.c
        hw_config.c
        glue.c
        )

target_compile_definitions(picow_ntp_client_background PRIVATE
        WIFI_SSID=\"Host\"
        WIFI_PASSWORD=\"secret\"
        )

target_include_directories(picow_ntp_client_background PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

# pull in common dependencies and additional i2c hardware support
target_link_libraries(picow_ntp_client_background
        pico_cyw43_arch_lwip_threadsafe_background
        pico_stdlib 
        hardware_i2c
        hardware_spi
        )

pico_add_extra_outputs(picow_ntp_client_background)

add_executable(picow_ntp_client_poll
        ssd1306_i2c_1.cpp
        WS2812.cpp
        sd_card.c
        ff.c
        ffsystem.c
        ffunicode.c
        spi.c
        sd_spi.c
        crc.c
        my_debug.c
        hw_config.c
        glue.c
        )

target_compile_definitions(picow_ntp_client_poll PRIVATE
        WIFI_SSID=\"Host\"
        WIFI_PASSWORD=\"secret\"
        )

target_include_directories(picow_ntp_client_poll PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )

target_link_libraries(picow_ntp_client_poll
        pico_cyw43_arch_lwip_poll
        pico_stdlib
        hardware_i2c
        hardware_spi
        )

pico_add_extra_outputs(picow_ntp_client_poll)


# Enable usb output, disable uart output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
