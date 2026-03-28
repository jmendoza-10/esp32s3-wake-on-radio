#include "strategy_dtim.h"

#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "wifi_connect.h"
#include "power_log.h"
#include "deep_sleep.h"

static const char *TAG = "dtim";

static bool wait_for_udp_trigger(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Socket creation failed");
        return false;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_DTIM_TRIGGER_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Bind failed");
        close(sock);
        return false;
    }

    ESP_LOGI(TAG, "Listening for UDP trigger on port %d", CONFIG_DTIM_TRIGGER_PORT);

    uint8_t buf[64];
    struct sockaddr_in src;
    socklen_t src_len = sizeof(src);
    int n = recvfrom(sock, buf, sizeof(buf), 0,
                     (struct sockaddr *)&src, &src_len);

    close(sock);

    if (n > 0) {
        ESP_LOGI(TAG, "Trigger received: %d bytes from " IPSTR,
                 n, IP2STR((esp_ip4_addr_t *)&src.sin_addr));
        return true;
    }

    return false;
}

void strategy_dtim_run(void)
{
    ESP_LOGI(TAG, "DTIM power save: listen_interval=%d, trigger_port=%d",
             CONFIG_DTIM_LISTEN_INTERVAL, CONFIG_DTIM_TRIGGER_PORT);

    /* Full Wi-Fi connect first */
    power_log_state(STATE_WIFI_CONNECTING);
    esp_err_t err = wifi_connect_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi connect failed — entering deep sleep");
        deep_sleep_enter_timed(10);
        return;
    }
    power_log_state(STATE_WIFI_CONNECTED);

    /* Set listen interval for power save */
    wifi_config_t cfg;
    esp_wifi_get_config(WIFI_IF_STA, &cfg);
    cfg.sta.listen_interval = CONFIG_DTIM_LISTEN_INTERVAL;
    esp_wifi_set_config(WIFI_IF_STA, &cfg);

    /* Enable maximum modem sleep — radio off between DTIM beacons */
    esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
    ESP_LOGI(TAG, "Modem power save enabled (MAX_MODEM)");

#ifdef CONFIG_PM_ENABLE
    /* Enable automatic light sleep when idle */
    esp_pm_config_t pm_cfg = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_XTAL_FREQ,
        .light_sleep_enable = true,
    };
    esp_pm_configure(&pm_cfg);
    ESP_LOGI(TAG, "Auto light sleep enabled");
#endif

    power_log_state(STATE_WIFI_PS_IDLE);

    /* Block until a UDP trigger arrives */
    if (wait_for_udp_trigger()) {
        power_log_state(STATE_TRIGGER_DETECTED);
        ESP_LOGI(TAG, "Trigger received — doing active work");
        power_log_state(STATE_ACTIVE);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }

    wifi_connect_deinit();
    power_log_state(STATE_PRE_SLEEP);
    power_log_summary();

    ESP_LOGI(TAG, "Returning to DTIM mode via deep sleep reset");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100));
    deep_sleep_enter_timed(1);
}
