include(../pico_sdk_import.cmake)

add_executable(my_blink
        my_blink.c
        )

# pull in common dependencies
target_link_libraries(my_blink 
                pico_stdlib              # for core functionality
                pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
)

# create map/bin/hex file etc.
pico_add_extra_outputs(my_blink)

# add url via pico_set_program_url
example_auto_set_url(my_blink)

pico_enable_stdio_usb(my_blink 0)
pico_enable_stdio_uart(my_blink 1)
