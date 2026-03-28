#pragma once

#include <stdint.h>

typedef enum {
    STATE_BOOT,
    STATE_WIFI_CONNECTING,
    STATE_WIFI_CONNECTED,
    STATE_ACTIVE,
    STATE_PRE_SLEEP,
    STATE_WAKEUP,
    /* Phase 2 */
    STATE_LIGHT_SLEEP,
    STATE_SCANNING,
    STATE_ESPNOW_LISTENING,
    STATE_BLE_SCANNING,
    STATE_TRIGGER_DETECTED,
    STATE_WIFI_PS_IDLE,
} power_state_t;

/* Log a state transition with a microsecond timestamp (from boot).
 * Printed over UART so the RPi serial logger can capture it. */
void power_log_state(power_state_t state);

/* Log an arbitrary measurement (e.g. from INA219 over I2C) */
void power_log_measurement(float voltage_mv, float current_ua);

/* Print a summary of all recorded state transitions and durations */
void power_log_summary(void);
