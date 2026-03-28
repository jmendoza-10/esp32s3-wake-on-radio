#pragma once

/* ESP-NOW broadcast wake strategy.
 * Listens for ESP-NOW frames on a fixed channel. Much faster than full
 * Wi-Fi scan — no association required to receive. */
void strategy_espnow_run(void);
