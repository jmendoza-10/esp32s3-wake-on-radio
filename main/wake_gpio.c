#include "wake_gpio.h"

#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "wake_gpio";

/*
 * Default: GPIO2 — on the same header side as GND and TX on the
 * ESP32-S3-DevKitC-1.  Override via Kconfig (CONFIG_WAKE_GPIO_NUM).
 */
#ifndef CONFIG_WAKE_GPIO_NUM
#define CONFIG_WAKE_GPIO_NUM 2
#endif

#define WAKE_GPIO  ((gpio_num_t)CONFIG_WAKE_GPIO_NUM)

esp_err_t wake_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << WAKE_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* keep LOW when floating */
        .intr_type    = GPIO_INTR_DISABLE,
    };

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO%d: %s",
                 WAKE_GPIO, esp_err_to_name(ret));
        return ret;
    }

    gpio_set_level(WAKE_GPIO, 0);

    /*
     * gpio_hold_en() latches the current output level so it persists
     * through deep sleep and the early boot phase (before app_main
     * runs).  This prevents the pin from floating HIGH and generating
     * spurious interrupts on the RPi.
     */
    gpio_hold_en(WAKE_GPIO);

    ESP_LOGI(TAG, "Wake GPIO%d initialised (output, LOW, held)", WAKE_GPIO);
    return ESP_OK;
}

void wake_gpio_assert(void)
{
    gpio_hold_dis(WAKE_GPIO);
    gpio_set_level(WAKE_GPIO, 1);
    gpio_hold_en(WAKE_GPIO);
    ESP_LOGI(TAG, "Wake GPIO%d → HIGH", WAKE_GPIO);
}

void wake_gpio_deassert(void)
{
    gpio_hold_dis(WAKE_GPIO);
    gpio_set_level(WAKE_GPIO, 0);
    gpio_hold_en(WAKE_GPIO);
    ESP_LOGI(TAG, "Wake GPIO%d → LOW", WAKE_GPIO);
}
