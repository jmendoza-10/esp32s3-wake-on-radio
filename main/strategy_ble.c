#include "strategy_ble.h"

#include <string.h>
#include "esp_log.h"
#include "esp_sleep.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "wifi_connect.h"
#include "power_log.h"
#include "deep_sleep.h"

static const char *TAG = "ble_wake";

#define TRIGGER_BIT BIT0

static EventGroupHandle_t s_ble_events;

static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    struct ble_hs_adv_fields fields;
    int rc = ble_hs_adv_parse_fields(&fields, event->disc.data,
                                      event->disc.length_data);
    if (rc != 0) {
        return 0;
    }

    if (fields.name != NULL && fields.name_len > 0) {
        if (fields.name_len == strlen(CONFIG_BLE_TRIGGER_NAME) &&
            memcmp(fields.name, CONFIG_BLE_TRIGGER_NAME,
                   fields.name_len) == 0) {
            ESP_LOGI(TAG, "BLE trigger detected: %.*s (RSSI %d)",
                     fields.name_len, fields.name, event->disc.rssi);
            xEventGroupSetBits(s_ble_events, TRIGGER_BIT);
            /* Stop scanning */
            ble_gap_disc_cancel();
        }
    }

    return 0;
}

static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void start_ble_scan(void)
{
    struct ble_gap_disc_params scan_params = {
        .itvl = BLE_GAP_SCAN_ITVL_MS(CONFIG_BLE_SCAN_WINDOW_MS),
        .window = BLE_GAP_SCAN_WIN_MS(CONFIG_BLE_SCAN_WINDOW_MS),
        .filter_policy = BLE_HCI_SCAN_FILT_NO_WL,
        .limited = 0,
        .passive = 1,  /* Passive scan — lower power, no probe requests */
        .filter_duplicates = 1,
    };

    ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER,
                 &scan_params, ble_gap_event_cb, NULL);
}

static void on_sync_cb(void)
{
    start_ble_scan();
}

void strategy_ble_run(void)
{
    ESP_LOGI(TAG, "BLE advertising wake: trigger_name=%s, window=%d ms, interval=%d ms",
             CONFIG_BLE_TRIGGER_NAME, CONFIG_BLE_SCAN_WINDOW_MS,
             CONFIG_BLE_SCAN_INTERVAL_MS);

    s_ble_events = xEventGroupCreate();

    /* Initialize NimBLE */
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = on_sync_cb;
    nimble_port_freertos_init(ble_host_task);

    int cycle = 0;
    while (1) {
        cycle++;
        power_log_state(STATE_BLE_SCANNING);
        ESP_LOGI(TAG, "BLE scan cycle %d", cycle);

        EventBits_t bits = xEventGroupWaitBits(s_ble_events,
            TRIGGER_BIT, pdTRUE, pdFALSE,
            pdMS_TO_TICKS(CONFIG_BLE_SCAN_WINDOW_MS));

        if (bits & TRIGGER_BIT) {
            power_log_state(STATE_TRIGGER_DETECTED);
            break;
        }

        /* No trigger — stop BLE and light sleep */
        ble_gap_disc_cancel();

        power_log_state(STATE_LIGHT_SLEEP);
        int sleep_ms = CONFIG_BLE_SCAN_INTERVAL_MS - CONFIG_BLE_SCAN_WINDOW_MS;
        if (sleep_ms < 10) sleep_ms = 10;
        esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
        esp_light_sleep_start();

        /* Resume scanning */
        start_ble_scan();
    }

    /* Trigger detected — shut down BLE, bring up Wi-Fi */
    nimble_port_stop();
    nimble_port_deinit();

    power_log_state(STATE_WIFI_CONNECTING);
    esp_err_t err = wifi_connect_init();
    if (err == ESP_OK) {
        power_log_state(STATE_WIFI_CONNECTED);
        ESP_LOGI(TAG, "Wi-Fi up — sending telemetry");
        power_log_state(STATE_ACTIVE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    wifi_connect_deinit();
    power_log_state(STATE_PRE_SLEEP);
    power_log_summary();

    ESP_LOGI(TAG, "Returning to scan loop via deep sleep reset");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    deep_sleep_enter_timed(1);
}
