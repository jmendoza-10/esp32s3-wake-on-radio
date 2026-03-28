#pragma once

/* BLE advertising wake strategy.
 * Scans for a BLE advertisement with a specific device name.
 * Much lower power than Wi-Fi scan (~10-15 mA vs ~100 mA).
 * Only brings up Wi-Fi after BLE trigger is detected. */
void strategy_ble_run(void);
