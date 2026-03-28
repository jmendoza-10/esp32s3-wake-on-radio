#pragma once

#include "esp_err.h"

/* Wakeup cause from the most recent boot */
const char *deep_sleep_wakeup_cause_str(void);

/* Enter deep sleep with a timer wakeup after `seconds` */
void deep_sleep_enter_timed(uint32_t seconds);

/* Enter deep sleep with EXT1 GPIO wakeup (for future use) */
void deep_sleep_enter_ext1(uint64_t gpio_mask);
