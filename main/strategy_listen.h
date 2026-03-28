#pragma once

/* Periodic listen windows strategy.
 * Wakes from light sleep every LISTEN_INTERVAL_MS, does a quick Wi-Fi scan
 * for LISTEN_MAGIC_SSID, and goes back to sleep if not found. */
void strategy_listen_run(void);
