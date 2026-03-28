#pragma once

/* Wi-Fi DTIM-based power save strategy.
 * Stays associated with the AP using 802.11 power save mode.
 * Wakes at DTIM beacons to check for buffered frames.
 * Lowest latency but highest average power. */
void strategy_dtim_run(void);
