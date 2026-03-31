#!/usr/bin/env bash
# Setup RPi for ESP32-S3 wake-on-radio power monitoring
# Configures I2C, UART, installs deps, and deploys the serial logger.
#
# Usage (from Mac Studio):
#   scp -r rpi/ pi@axon-command.local:~/wake-on-radio/
#   ssh pi@axon-command.local 'bash ~/wake-on-radio/setup_rpi.sh'
#
# Or all-in-one:
#   ./scripts/deploy_rpi.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
NEEDS_REBOOT=false

echo "=== ESP32-S3 Wake-on-Radio — RPi Setup ==="
echo ""

# ── Enable I2C ──────────────────────────────────────────────────────────────
if ! grep -q "^dtparam=i2c_arm=on" /boot/firmware/config.txt 2>/dev/null; then
    echo "[1/4] Enabling I2C..."
    sudo raspi-config nonint do_i2c 0
    NEEDS_REBOOT=true
else
    echo "[1/4] I2C already enabled"
fi

# ── Enable hardware UART, disable console on serial ────────────────────────
if grep -q "console=serial0" /boot/firmware/cmdline.txt 2>/dev/null; then
    echo "[2/4] Configuring UART (enable hw serial, disable console)..."
    sudo raspi-config nonint do_serial_hw 0
    sudo raspi-config nonint do_serial_cons 1
    NEEDS_REBOOT=true
else
    echo "[2/4] UART already configured"
fi

# ── Install system packages ────────────────────────────────────────────────
echo "[3/6] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y -qq i2c-tools python3-pip python3-venv \
    raspberrypi-kernel-headers device-tree-compiler \
    sigrok sigrok-firmware-fx2lafw

# ── Build kernel module ───────────────────────────────────────────────────
DRIVER_DIR="$SCRIPT_DIR/driver"
if [ -d "$DRIVER_DIR" ] && [ -f "$DRIVER_DIR/esp32_wor.c" ]; then
    echo "[4/6] Building esp32_wor kernel module..."
    make -C "$DRIVER_DIR" clean || true
    make -C "$DRIVER_DIR"

    # Install into kernel modules tree so modprobe/depmod can find it
    KVER=$(uname -r)
    INSTALL_DIR="/lib/modules/$KVER/extra"
    sudo mkdir -p "$INSTALL_DIR"
    sudo cp "$DRIVER_DIR/esp32_wor.ko" "$INSTALL_DIR/"
    sudo depmod -a
    echo "  Module installed: $INSTALL_DIR/esp32_wor.ko"
else
    echo "[4/6] SKIP — driver sources not found in $DRIVER_DIR"
fi

# ── Compile and install Device Tree overlay ───────────────────────────────
if [ -f "$DRIVER_DIR/esp32-wor-overlay.dts" ]; then
    echo "[5/6] Compiling Device Tree overlay..."
    dtc -@ -I dts -O dtb -o "$DRIVER_DIR/esp32-wor.dtbo" \
        "$DRIVER_DIR/esp32-wor-overlay.dts" 2>/dev/null
    sudo cp "$DRIVER_DIR/esp32-wor.dtbo" /boot/overlays/

    # Auto-load module at boot
    if [ ! -f /etc/modules-load.d/esp32-wor.conf ]; then
        echo "esp32_wor" | sudo tee /etc/modules-load.d/esp32-wor.conf > /dev/null
        echo "  Auto-load configured via /etc/modules-load.d/esp32-wor.conf"
    fi

    # Add overlay to config.txt if not already present
    CONFIG_FILE="/boot/firmware/config.txt"
    [ ! -f "$CONFIG_FILE" ] && CONFIG_FILE="/boot/config.txt"
    if ! grep -q "^dtoverlay=esp32-wor" "$CONFIG_FILE" 2>/dev/null; then
        echo "dtoverlay=esp32-wor" | sudo tee -a "$CONFIG_FILE" > /dev/null
        echo "  Added dtoverlay=esp32-wor to $CONFIG_FILE"
        NEEDS_REBOOT=true
    else
        echo "  Overlay already in $CONFIG_FILE"
    fi
else
    echo "[5/6] SKIP — overlay DTS not found"
fi

# ── Python venv + deps ─────────────────────────────────────────────────────
echo "[6/6] Setting up Python environment..."
VENV_DIR="$SCRIPT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi
"$VENV_DIR/bin/pip" install --quiet pyserial smbus2 flask

# ── Install systemd service ────────────────────────────────────────────────
if [ -f "$SCRIPT_DIR/esp32-wor-dashboard.service" ]; then
    echo "Installing dashboard systemd service..."
    sudo cp "$SCRIPT_DIR/esp32-wor-dashboard.service" /etc/systemd/system/
    sudo systemctl daemon-reload
    sudo systemctl enable esp32-wor-dashboard.service
    sudo systemctl restart esp32-wor-dashboard.service
    echo "  Service enabled and started (port 8080)"
fi

echo ""
echo "=== Setup complete ==="

# ── Verify hardware ────────────────────────────────────────────────────────
if [ "$NEEDS_REBOOT" = true ]; then
    echo ""
    echo "*** Reboot required for I2C/UART changes. ***"
    read -p "Reboot now? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Rebooting... SSH back in and run:"
        echo "  cd ~/wake-on-radio && ./verify.sh"
        sudo reboot
    else
        echo "Run 'sudo reboot' when ready, then:"
        echo "  cd ~/wake-on-radio && ./verify.sh"
    fi
else
    echo ""
    bash "$SCRIPT_DIR/verify.sh"
fi
