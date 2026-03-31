#pragma once

/*
 * wake_gpio — drives a GPIO pin HIGH when a wake trigger is detected.
 *
 * The RPi Linux driver (esp32_wor.ko) watches this pin as an interrupt
 * source, giving it a hardware-level wake notification with microsecond
 * latency instead of relying on UART parsing.
 *
 * The pin is active-high:
 *   - LOW  = idle / no trigger
 *   - HIGH = trigger detected, ESP32 is awake and active
 *
 * Call wake_gpio_init() once at boot, then wake_gpio_assert() when a
 * trigger fires and wake_gpio_deassert() before going back to sleep.
 */

#include "esp_err.h"

/* Initialise the wake GPIO pin as output, driven LOW. */
esp_err_t wake_gpio_init(void);

/* Drive the pin HIGH — call when TRIGGER_DETECTED. */
void wake_gpio_assert(void);

/* Drive the pin LOW — call before entering sleep / idle. */
void wake_gpio_deassert(void);
