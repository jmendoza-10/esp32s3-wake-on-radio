#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "deep_sleep.h"
#include "wifi_connect.h"
#include "power_log.h"

#if defined(CONFIG_WAKE_STRATEGY_LISTEN)
#include "strategy_listen.h"
#elif defined(CONFIG_WAKE_STRATEGY_ESPNOW)
#include "strategy_espnow.h"
#elif defined(CONFIG_WAKE_STRATEGY_BLE)
#include "strategy_ble.h"
#elif defined(CONFIG_WAKE_STRATEGY_DTIM)
#include "strategy_dtim.h"
#endif

static const char *TAG = "main";

/* Deep sleep duration between wake cycles (seconds) — baseline only */
#define SLEEP_DURATION_S  10

static void baseline_run(void)
{
    power_log_state(STATE_WIFI_CONNECTING);

    esp_err_t err = wifi_connect_init();
    if (err == ESP_OK) {
        power_log_state(STATE_WIFI_CONNECTED);
        ESP_LOGI(TAG, "Wi-Fi up — would send telemetry here");

        power_log_state(STATE_ACTIVE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
        ESP_LOGW(TAG, "Wi-Fi failed — skipping telemetry");
    }

    wifi_connect_deinit();
    power_log_state(STATE_PRE_SLEEP);

    power_log_summary();

    ESP_LOGI(TAG, "Going to deep sleep for %d s ...", SLEEP_DURATION_S);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));

    deep_sleep_enter_timed(SLEEP_DURATION_S);
}

void app_main(void)
{
    power_log_state(STATE_BOOT);

    const char *cause = deep_sleep_wakeup_cause_str();

    const char *strategy =
#if defined(CONFIG_WAKE_STRATEGY_LISTEN)
        "Periodic Listen";
#elif defined(CONFIG_WAKE_STRATEGY_ESPNOW)
        "ESP-NOW Wake";
#elif defined(CONFIG_WAKE_STRATEGY_BLE)
        "BLE Advertising Wake";
#elif defined(CONFIG_WAKE_STRATEGY_DTIM)
        "DTIM Power Save";
#else
        "Baseline (deep sleep + timer)";
#endif

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Wake-on-Radio — %s", strategy);
    ESP_LOGI(TAG, " Wakeup cause: %s", cause);
    ESP_LOGI(TAG, "========================================");

    power_log_state(STATE_WAKEUP);

#if defined(CONFIG_WAKE_STRATEGY_LISTEN)
    strategy_listen_run();
#elif defined(CONFIG_WAKE_STRATEGY_ESPNOW)
    strategy_espnow_run();
#elif defined(CONFIG_WAKE_STRATEGY_BLE)
    strategy_ble_run();
#elif defined(CONFIG_WAKE_STRATEGY_DTIM)
    strategy_dtim_run();
#else
    baseline_run();
#endif
}
