#include "deep_sleep.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"

static const char *TAG = "deep_sleep";

/* Map individual wakeup cause bits to names */
static const struct {
    esp_sleep_wakeup_cause_t bit;
    const char *name;
} wakeup_cause_map[] = {
    { ESP_SLEEP_WAKEUP_TIMER,    "TIMER" },
    { ESP_SLEEP_WAKEUP_EXT0,     "EXT0" },
    { ESP_SLEEP_WAKEUP_EXT1,     "EXT1" },
    { ESP_SLEEP_WAKEUP_TOUCHPAD, "TOUCHPAD" },
    { ESP_SLEEP_WAKEUP_ULP,      "ULP" },
    { ESP_SLEEP_WAKEUP_GPIO,     "GPIO" },
    { ESP_SLEEP_WAKEUP_UART,     "UART" },
};

const char *deep_sleep_wakeup_cause_str(void)
{
    static char buf[64];
    uint32_t causes = esp_sleep_get_wakeup_causes();

    if (causes == 0) {
        return "POWER_ON";
    }

    buf[0] = '\0';
    for (size_t i = 0; i < sizeof(wakeup_cause_map) / sizeof(wakeup_cause_map[0]); i++) {
        if (causes & (uint32_t)wakeup_cause_map[i].bit) {
            if (buf[0] != '\0') {
                strlcat(buf, "+", sizeof(buf));
            }
            strlcat(buf, wakeup_cause_map[i].name, sizeof(buf));
        }
    }

    return buf[0] ? buf : "UNKNOWN";
}

void deep_sleep_enter_timed(uint32_t seconds)
{
    ESP_LOGI(TAG, "Entering deep sleep for %"PRIu32" s", seconds);

    /* Isolate all GPIOs to minimize leakage current.
     * On ESP32-S3 this disconnects the pads from digital logic. */
    esp_sleep_config_gpio_isolate();

    /* Disable pull-ups/pull-downs on RTC GPIOs to shave ~1-2 uA */
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);

    esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
    esp_deep_sleep_start();
    /* Does not return */
}

void deep_sleep_enter_ext1(uint64_t gpio_mask)
{
    ESP_LOGI(TAG, "Entering deep sleep with EXT1 mask 0x%llx", gpio_mask);

    esp_sleep_config_gpio_isolate();
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);

    esp_sleep_enable_ext1_wakeup(gpio_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
    esp_deep_sleep_start();
}
