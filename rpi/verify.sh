#!/usr/bin/env bash
# Verify RPi hardware is ready for power monitoring
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
export PATH="$PATH:/usr/sbin"
OK=true

echo "=== Hardware Verification ==="
echo ""

# Check I2C
echo -n "I2C bus:    "
if [ -e /dev/i2c-1 ] && i2cdetect -y 1 > /dev/null 2>&1; then
    ADDRS=$(i2cdetect -y 1 2>/dev/null | grep -oP '(?<=\s)[0-9a-f]{2}(?=\s)' | tr '\n' ' ')
    if [ -n "$ADDRS" ]; then
        echo "OK — devices at: $ADDRS"
    else
        echo "WARN — bus active but no devices found. Check HAT seating."
        OK=false
    fi
else
    echo "FAIL — I2C not enabled. Run setup_rpi.sh and reboot."
    OK=false
fi

# Check INA219 on expected addresses
echo -n "INA219 CH1: "
if echo "$ADDRS" | grep -q "40"; then
    echo "OK — found at 0x40"
else
    echo "WARN — not found at 0x40. Check HAT channel 1 wiring."
    OK=false
fi

# Check UART — RPi 4 Bookworm may use ttyS0 instead of ttyAMA0
echo -n "UART:       "
if [ -e /dev/ttyAMA0 ]; then
    echo "OK — /dev/ttyAMA0"
elif [ -e /dev/ttyS0 ]; then
    echo "OK — /dev/ttyS0 (mini UART)"
else
    echo "FAIL — no UART device found. Run setup_rpi.sh and reboot."
    OK=false
fi

# Check serial console is disabled
echo -n "Console:    "
CMDLINE="/boot/firmware/cmdline.txt"
[ ! -f "$CMDLINE" ] && CMDLINE="/boot/cmdline.txt"
if grep -q "console=serial0\|console=ttyAMA0\|console=ttyS0" "$CMDLINE" 2>/dev/null; then
    echo "WARN — serial console still enabled (will conflict with ESP32 UART)"
    OK=false
else
    echo "OK — serial console disabled"
fi

# Check kernel module
echo -n "Driver:     "
DRIVER_DIR="$SCRIPT_DIR/driver"
if lsmod | grep -q esp32_wor; then
    echo "OK — esp32_wor module loaded"
elif [ -f "$DRIVER_DIR/esp32_wor.ko" ]; then
    echo "WARN — module built but not loaded. Load with:"
    echo "              sudo insmod $DRIVER_DIR/esp32_wor.ko"
    OK=false
else
    echo "WARN — module not built. Run setup_rpi.sh first."
    OK=false
fi

# Check DT overlay
echo -n "Overlay:    "
if [ -f /boot/overlays/esp32-wor.dtbo ]; then
    echo "OK — /boot/overlays/esp32-wor.dtbo installed"
else
    echo "WARN — overlay not installed. Run setup_rpi.sh first."
    OK=false
fi

# Check sysfs (only if module is loaded)
echo -n "Sysfs:      "
if [ -d /sys/devices/platform/esp32-wor ]; then
    WC=$(cat /sys/devices/platform/esp32-wor/wake_count 2>/dev/null || echo "?")
    ACT=$(cat /sys/devices/platform/esp32-wor/active 2>/dev/null || echo "?")
    echo "OK — wake_count=$WC active=$ACT"
elif lsmod | grep -q esp32_wor; then
    echo "WARN — module loaded but sysfs not found (reboot needed for overlay?)"
    OK=false
else
    echo "SKIP — module not loaded"
fi

# Check Python deps
echo -n "Python:     "
VENV="$SCRIPT_DIR/.venv/bin/python3"
if [ -x "$VENV" ] && "$VENV" -c "import serial, smbus2" 2>/dev/null; then
    echo "OK — pyserial + smbus2 installed"
else
    echo "FAIL — run setup_rpi.sh to install deps"
    OK=false
fi

echo ""
if [ "$OK" = true ]; then
    # Determine the right serial port for the logger
    UART_PORT="/dev/ttyAMA0"
    [ ! -e "$UART_PORT" ] && UART_PORT="/dev/ttyS0"
    echo "All checks passed."
    if ! lsmod | grep -q esp32_wor; then
        echo ""
        echo "Load the driver with:"
        echo "  sudo insmod ~/wake-on-radio/driver/esp32_wor.ko"
    fi
    echo ""
    echo "Start logging with:"
    echo "  cd ~/wake-on-radio && .venv/bin/python3 serial_logger.py --port $UART_PORT"
    echo ""
    echo "Monitor wake events:"
    echo "  cat /sys/devices/platform/esp32-wor/wake_count"
    echo "  cat /sys/devices/platform/esp32-wor/active"
else
    echo "Some checks failed — see above."
fi
