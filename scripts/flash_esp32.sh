#!/usr/bin/env bash
# Flash ESP32-S3 with wake-on-radio firmware
# Requires ESP-IDF to be installed: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/get-started/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

usage() {
    echo "Usage: $0 <baseline|listen|espnow|ble|dtim> [options]"
    echo ""
    echo "Firmware targets (wake strategies):"
    echo "  baseline  - Deep sleep with timer wakeup (Phase 1)"
    echo "  listen    - Periodic listen windows (Phase 2)"
    echo "  espnow    - ESP-NOW broadcast wake (Phase 2)"
    echo "  ble       - BLE advertising wake (Phase 2)"
    echo "  dtim      - Wi-Fi DTIM-based power save (Phase 2)"
    echo ""
    echo "Required:"
    echo "  --wifi-ssid <ssid>  WiFi network name"
    echo "  --wifi-pass <pass>  WiFi password"
    echo ""
    echo "Optional:"
    echo "  --port <port>              Serial port (auto-detected if not specified)"
    echo "  --sleep-duration <secs>    Deep sleep duration in seconds (default: 10)"
    echo "  --no-monitor               Skip the serial monitor prompt after flashing"
    echo ""
    echo "Prerequisites:"
    echo "  1. ESP-IDF installed and sourced:  . ~/workspace/opensource/esp/esp-idf/export.sh"
    echo "  2. ESP32-S3 connected via USB"
    echo ""
    echo "Examples:"
    echo "  # Phase 1 baseline deep sleep"
    echo "  $0 baseline --wifi-ssid MyWiFi --wifi-pass secret"
    echo ""
    echo "  # Specify port and sleep duration"
    echo "  $0 baseline --wifi-ssid MyWiFi --wifi-pass secret --port /dev/cu.usbmodem1234 --sleep-duration 30"
    echo ""
    echo "  # Flash and skip monitor prompt"
    echo "  $0 baseline --wifi-ssid MyWiFi --wifi-pass secret --no-monitor"
    exit 0
}

if [ $# -lt 1 ] || [[ "$1" == "--help" ]] || [[ "$1" == "-h" ]]; then
    usage
fi

FIRMWARE="$1"
shift

PORT=""
WIFI_SSID=""
WIFI_PASS=""
SLEEP_DURATION="10"
NO_MONITOR=false

# Parse named arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --port)            PORT="$2";            shift 2 ;;
        --wifi-ssid)       WIFI_SSID="$2";       shift 2 ;;
        --wifi-pass)       WIFI_PASS="$2";       shift 2 ;;
        --sleep-duration)  SLEEP_DURATION="$2";  shift 2 ;;
        --no-monitor)      NO_MONITOR=true;      shift ;;
        *) echo "Unknown argument: $1"; usage ;;
    esac
done

# Validate firmware target
case "$FIRMWARE" in
    baseline|listen|espnow|ble|dtim) ;;
    *) echo "Error: Unknown firmware target '$FIRMWARE'"; usage ;;
esac

# WiFi credentials are required for all strategies
if [ -z "$WIFI_SSID" ] || [ -z "$WIFI_PASS" ]; then
    echo "Error: --wifi-ssid and --wifi-pass are required."
    echo ""
    echo "  $0 $FIRMWARE --wifi-ssid <SSID> --wifi-pass <PASSWORD>"
    exit 1
fi

# Auto-detect port if not specified
if [ -z "$PORT" ]; then
    if [[ "$(uname -s)" == "Darwin" ]]; then
        PORT=$(ls /dev/cu.usbmodem* /dev/cu.SLAB_USBtoUART* /dev/cu.usbserial-* 2>/dev/null | head -1 || true)
    else
        PORT=$(ls /dev/ttyACM0 /dev/ttyUSB0 2>/dev/null | head -1 || true)
    fi

    if [ -z "$PORT" ]; then
        echo "Error: No ESP32 serial port found."
        echo ""
        if [[ "$(uname -s)" == "Darwin" ]]; then
            echo "Available ports:"
            ls /dev/cu.* 2>/dev/null | grep -v Bluetooth || echo "  (none)"
        else
            echo "Available ports:"
            ls /dev/tty{ACM,USB}* 2>/dev/null || echo "  (none)"
        fi
        echo ""
        echo "Connect the ESP32-S3 via USB and retry, or specify the port:"
        echo "  $0 $FIRMWARE --port /dev/cu.usbmodem1234"
        exit 1
    fi

    echo "Auto-detected port: $PORT"
fi

# Check ESP-IDF
if [ -z "${IDF_PATH:-}" ]; then
    echo "Error: ESP-IDF not found. Run: . ~/workspace/opensource/esp/esp-idf/export.sh"
    exit 1
fi

echo "=== Flashing wake-on-radio [$FIRMWARE] to $PORT ==="
cd "$PROJECT_DIR"

# Inject WiFi credentials and sleep duration into sdkconfig.defaults
DEFAULTS_FILE="sdkconfig.defaults"
EXISTING=$(grep -v "^CONFIG_WIFI_SSID\|^CONFIG_WIFI_PASSWORD\|^CONFIG_WAKE_SLEEP_DURATION" "$DEFAULTS_FILE" 2>/dev/null || true)
{
    echo "$EXISTING"
    echo "CONFIG_WIFI_SSID=\"${WIFI_SSID}\""
    echo "CONFIG_WIFI_PASSWORD=\"${WIFI_PASS}\""
} > "$DEFAULTS_FILE.tmp" && mv "$DEFAULTS_FILE.tmp" "$DEFAULTS_FILE"

echo "Config: ssid=$WIFI_SSID  sleep=${SLEEP_DURATION}s  strategy=$FIRMWARE"

# Append strategy-specific sdkconfig defaults
STRATEGY_DEFAULTS="sdkconfig.defaults.${FIRMWARE}"
if [ -f "$STRATEGY_DEFAULTS" ] && [ "$FIRMWARE" != "baseline" ]; then
    echo "Appending $STRATEGY_DEFAULTS"
    cat "$STRATEGY_DEFAULTS" >> "$DEFAULTS_FILE"
fi

# Set target to ESP32-S3 and clean (ensures sdkconfig.defaults are applied)
idf.py set-target esp32s3
idf.py fullclean

# Build
echo "Building firmware..."
idf.py build

# Flash
echo "Flashing to $PORT..."
idf.py -p "$PORT" flash

echo ""
echo "=== Flash complete! ==="
echo "  Strategy:       $FIRMWARE"
echo "  Sleep duration: ${SLEEP_DURATION}s"
echo "  Port:           $PORT"
echo ""
echo "RPi logger: python3 rpi/serial_logger.py --port /dev/ttyUSB0"

if [ "$NO_MONITOR" = false ]; then
    echo ""
    echo "To monitor serial output: idf.py -p $PORT monitor"
    read -p "Start serial monitor now? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        idf.py -p "$PORT" monitor
    fi
fi
