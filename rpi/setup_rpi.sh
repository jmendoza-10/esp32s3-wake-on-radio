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
echo "[3/4] Installing system packages..."
sudo apt-get update -qq
sudo apt-get install -y -qq i2c-tools python3-pip python3-venv

# ── Python venv + deps ─────────────────────────────────────────────────────
echo "[4/4] Setting up Python environment..."
VENV_DIR="$SCRIPT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    python3 -m venv "$VENV_DIR"
fi
"$VENV_DIR/bin/pip" install --quiet pyserial smbus2 flask

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
