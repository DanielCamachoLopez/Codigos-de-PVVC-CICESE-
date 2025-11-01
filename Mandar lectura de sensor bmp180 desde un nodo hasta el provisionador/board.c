/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"

#define TAG "BOARD"

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        /* Defensive: if software believes the LED is already in the desired state,
           verify the physical GPIO level. If it's desynced, force the GPIO and
           resync the software state. This prevents a stale `previous` from
           stopping further updates. */
        if (onoff == led_state[i].previous) {
            int phys = gpio_get_level(pin);
            if (phys != onoff) {
                ESP_LOGW(TAG, "led %s: software previous=%d but gpio level=%d, forcing to %d",
                         led_state[i].name, led_state[i].previous, phys, onoff);
                gpio_set_level(pin, onoff);
                led_state[i].previous = onoff;
            } else {
                ESP_LOGW(TAG, "led %s is already %s (phys=%d)",
                         led_state[i].name, (onoff ? "on" : "off"), phys);
            }
            return;
        }
        /* Normal path: change GPIO and update previous/current */
        ESP_LOGI(TAG, "led %s: changing from %d to %d", led_state[i].name, led_state[i].previous, onoff);
        gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;
        led_state[i].current = onoff;
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        /* Read back physical level to synchronize software state in case the
           pin behaves as active-low or was changed earlier in boot. */
        int phys = gpio_get_level(led_state[i].pin);
        led_state[i].previous = (phys != 0) ? LED_ON : LED_OFF;
        led_state[i].current = led_state[i].previous;
    }
}

void board_init(void)
{
    board_led_init();
}
