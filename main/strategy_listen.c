#include "strategy_listen.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "wifi_connect.h"
#include "power_log.h"
#include "deep_sleep.h"

static const char *TAG = "listen";

static bool scan_for_trigger(void)
{
    wifi_scan_config_t scan_cfg = {
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 20,
        .scan_time.active.max = CONFIG_LISTEN_DURATION_MS,
#if CONFIG_LISTEN_SCAN_CHANNEL > 0
        .channel = CONFIG_LISTEN_SCAN_CHANNEL,
#endif
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Scan start failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t count = 0;
    esp_wifi_scan_get_ap_num(&count);
    if (count == 0) {
        esp_wifi_clear_ap_list();
        return false;
    }

    wifi_ap_record_t *records = malloc(count * sizeof(wifi_ap_record_t));
    if (!records) {
        esp_wifi_clear_ap_list();
        return false;
    }

    esp_wifi_scan_get_ap_records(&count, records);

    bool found = false;
    for (int i = 0; i < count; i++) {
        if (strcmp((const char *)records[i].ssid, CONFIG_LISTEN_MAGIC_SSID) == 0) {
            ESP_LOGI(TAG, "Trigger SSID found: %s (RSSI %d)",
                     records[i].ssid, records[i].rssi);
            found = true;
            break;
        }
    }

    free(records);
    return found;
}

void strategy_listen_run(void)
{
    ESP_LOGI(TAG, "Periodic listen: interval=%d ms, duration=%d ms, ssid=%s",
             CONFIG_LISTEN_INTERVAL_MS, CONFIG_LISTEN_DURATION_MS,
             CONFIG_LISTEN_MAGIC_SSID);

    wifi_start_no_connect();

    int cycle = 0;
    while (1) {
        cycle++;
        power_log_state(STATE_SCANNING);
        ESP_LOGI(TAG, "Scan cycle %d", cycle);

        if (scan_for_trigger()) {
            power_log_state(STATE_TRIGGER_DETECTED);
            break;
        }

        /* Stop Wi-Fi radio before sleeping to save power */
        esp_wifi_stop();

        power_log_state(STATE_LIGHT_SLEEP);
        esp_sleep_enable_timer_wakeup(
            (uint64_t)CONFIG_LISTEN_INTERVAL_MS * 1000ULL);
        esp_light_sleep_start();

        /* Re-enable Wi-Fi for next scan */
        esp_wifi_start();
    }

    /* Trigger detected — do full Wi-Fi connect and active work */
    power_log_state(STATE_WIFI_CONNECTING);
    esp_err_t err = wifi_connect_start();
    if (err == ESP_OK) {
        power_log_state(STATE_WIFI_CONNECTED);
        ESP_LOGI(TAG, "Wi-Fi up — sending telemetry");
        power_log_state(STATE_ACTIVE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    wifi_connect_deinit();
    power_log_state(STATE_PRE_SLEEP);
    power_log_summary();

    ESP_LOGI(TAG, "Returning to listen loop via deep sleep reset");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    deep_sleep_enter_timed(1);
}
