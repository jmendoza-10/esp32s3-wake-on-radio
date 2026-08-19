#!/usr/bin/env bash
set -euo pipefail

cd /home/axon/wake-on-radio
# Set --shunt-ohms to the installed Waveshare sense resistor value:
# stock Waveshare = 0.1, 20 mOhm modification = 0.02.
exec .venv/bin/python3 dashboard.py \
  --port /dev/ttyS0 \
  --web-port 8080 \
  --sample-rate 1000 \
  --ina-channel 1 \
  --ina-channels 1,2,3,4 \
  --highspeed-channels 1,2 \
  --shunt-ohms 0.1 \
  --max-current 3.0 \
  --external-monitor-type auto \
  --external-monitor-address 0x44 \
  --ina226-shunt-ohms 0.002 \
  --ina226-max-current 6.0 \
  --ina226-sample-rate 1000 \
  --ina226-averages 64 \
  --ina226-conversion-time-us 1100 \
  --ina228-adc-range-mv 40.96 \
  --ina226-rail 4v \
  --out /dev/null
