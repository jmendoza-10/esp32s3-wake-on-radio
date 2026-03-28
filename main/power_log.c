#include "power_log.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "power_log";

#define MAX_LOG_ENTRIES 64

typedef struct {
    int64_t       timestamp_us;
    power_state_t state;
} log_entry_t;

static log_entry_t s_log[MAX_LOG_ENTRIES];
static int s_log_count;

static const char *state_name(power_state_t s)
{
    switch (s) {
    case STATE_BOOT:              return "BOOT";
    case STATE_WIFI_CONNECTING:   return "WIFI_CONNECTING";
    case STATE_WIFI_CONNECTED:    return "WIFI_CONNECTED";
    case STATE_ACTIVE:            return "ACTIVE";
    case STATE_PRE_SLEEP:         return "PRE_SLEEP";
    case STATE_WAKEUP:            return "WAKEUP";
    case STATE_LIGHT_SLEEP:       return "LIGHT_SLEEP";
    case STATE_SCANNING:          return "SCANNING";
    case STATE_ESPNOW_LISTENING:  return "ESPNOW_LISTENING";
    case STATE_BLE_SCANNING:      return "BLE_SCANNING";
    case STATE_TRIGGER_DETECTED:  return "TRIGGER_DETECTED";
    case STATE_WIFI_PS_IDLE:      return "WIFI_PS_IDLE";
    default:                      return "UNKNOWN";
    }
}

void power_log_state(power_state_t state)
{
    int64_t now = esp_timer_get_time();

    if (s_log_count < MAX_LOG_ENTRIES) {
        s_log[s_log_count].timestamp_us = now;
        s_log[s_log_count].state = state;
        s_log_count++;
    }

    /* Machine-parseable line for the RPi logger:
     * PWR|<timestamp_us>|<state_name> */
    printf("PWR|%lld|%s\n", now, state_name(state));
}

void power_log_measurement(float voltage_mv, float current_ua)
{
    int64_t now = esp_timer_get_time();
    printf("MEAS|%lld|%.1f|%.1f\n", now, voltage_mv, current_ua);
}

void power_log_summary(void)
{
    ESP_LOGI(TAG, "=== State Transition Summary ===");
    for (int i = 0; i < s_log_count; i++) {
        int64_t delta = 0;
        if (i > 0) {
            delta = s_log[i].timestamp_us - s_log[i - 1].timestamp_us;
        }
        ESP_LOGI(TAG, "  %-18s @ %8lld us  (delta %lld us)",
                 state_name(s_log[i].state),
                 s_log[i].timestamp_us,
                 delta);
    }
    ESP_LOGI(TAG, "================================");
}
