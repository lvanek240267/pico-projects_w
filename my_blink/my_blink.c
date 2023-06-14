/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h" 
#include "pico/cyw43_arch.h"

int main() 
{
    stdio_init_all();
    if (cyw43_arch_init()) 
    {
        printf("Wi-Fi init failed");
        return -1;
    }

    int a = 0;
    int b = 10000;

    while (true) 
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);

        a++;
        b--;
        printf("a = %d, b = %d\n", a, b);

        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
}


