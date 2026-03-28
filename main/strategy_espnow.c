#include "strategy_espnow.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "wifi_connect.h"
#include "power_log.h"
#include "deep_sleep.h"

static const char *TAG = "espnow";

#define TRIGGER_BIT BIT0

static EventGroupHandle_t s_espnow_events;

static void espnow_recv_cb(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len)
{
    if (len >= 1 && data[0] == CONFIG_ESPNOW_MAGIC) {
        ESP_LOGI(TAG, "Trigger received from " MACSTR " (len=%d)",
                 MAC2STR(info->src_addr), len);
        xEventGroupSetBits(s_espnow_events, TRIGGER_BIT);
    }
}

static esp_err_t espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    /* Add broadcast peer so we can receive broadcast frames */
    esp_now_peer_info_t peer = {
        .channel = CONFIG_ESPNOW_CHANNEL,
        .ifidx = WIFI_IF_STA,
        .encrypt = false,
    };
    memset(peer.peer_addr, 0xFF, ESP_NOW_ETH_ALEN);
    esp_now_add_peer(&peer);

    return ESP_OK;
}

void strategy_espnow_run(void)
{
    ESP_LOGI(TAG, "ESP-NOW wake: channel=%d, interval=%d ms, window=%d ms",
             CONFIG_ESPNOW_CHANNEL, CONFIG_LISTEN_INTERVAL_MS,
             CONFIG_LISTEN_DURATION_MS);

    s_espnow_events = xEventGroupCreate();

    wifi_start_no_connect();

    /* Fix to the ESP-NOW channel */
    esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

    espnow_init();

    int cycle = 0;
    while (1) {
        cycle++;
        power_log_state(STATE_ESPNOW_LISTENING);
        ESP_LOGI(TAG, "Listen cycle %d", cycle);

        /* Wait for trigger during the listen window */
        EventBits_t bits = xEventGroupWaitBits(s_espnow_events,
            TRIGGER_BIT, pdTRUE, pdFALSE,
            pdMS_TO_TICKS(CONFIG_LISTEN_DURATION_MS));

        if (bits & TRIGGER_BIT) {
            power_log_state(STATE_TRIGGER_DETECTED);
            break;
        }

        /* No trigger — stop radio and light sleep */
        esp_now_deinit();
        esp_wifi_stop();

        power_log_state(STATE_LIGHT_SLEEP);
        esp_sleep_enable_timer_wakeup(
            (uint64_t)CONFIG_LISTEN_INTERVAL_MS * 1000ULL);
        esp_light_sleep_start();

        /* Re-enable for next cycle */
        esp_wifi_start();
        esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
        espnow_init();
    }

    /* Trigger detected — full Wi-Fi connect */
    esp_now_deinit();

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
