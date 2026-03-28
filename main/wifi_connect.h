#pragma once

#include "esp_err.h"

/* Initialise NVS, netif, and event loop. Safe to call multiple times. */
esp_err_t wifi_system_init(void);

/* Start Wi-Fi STA and connect to the configured AP.
 * Blocks until connected or timeout (10 s). Calls wifi_system_init(). */
esp_err_t wifi_connect_init(void);

/* Start Wi-Fi STA without connecting (for scan/ESP-NOW use).
 * Calls wifi_system_init(). */
esp_err_t wifi_start_no_connect(void);

/* Connect to the configured AP after wifi_start_no_connect().
 * Blocks until connected or timeout. */
esp_err_t wifi_connect_start(void);

/* Disconnect and release Wi-Fi resources to prepare for sleep. */
void wifi_connect_deinit(void);

/* Returns true when an IP has been acquired. */
bool wifi_is_connected(void);
