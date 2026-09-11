# Power Monitor Wiring Reference

Use this reference when wiring the Raspberry Pi power monitors used by the
wake-on-radio dashboard. Power everything off before adding, removing, or
moving sense wires.

## Raspberry Pi I2C pins

Both monitors share Raspberry Pi I2C bus 1.

| Signal | Raspberry Pi physical pin | GPIO |
|---|---:|---|
| 3.3 V | 1 or 17 | - |
| SDA | 3 | GPIO2 / SDA1 |
| SCL | 5 | GPIO3 / SCL1 |
| GND | 6, 9, or 14 | - |

For this bench, power monitor logic from **3.3 V, not 5 V**. All monitor and
DUT grounds must share the same reference. These monitors are not galvanically
isolated.

## Waveshare 4-channel HAT (INA219)

The HAT plugs directly onto the Raspberry Pi 40-pin header. Each channel has
its own onboard 0.1 ohm shunt resistor.

| HAT channel | I2C address | Sense terminals |
|---|---:|---|
| CH1 | `0x40` | `IN1+`, `IN1-`, `GND` |
| CH2 | `0x41` | `IN2+`, `IN2-`, `GND` |
| CH3 | `0x42` | `IN3+`, `IN3-`, `GND` |
| CH4 | `0x43` | `IN4+`, `IN4-`, `GND` |

Wire every channel as a high-side series measurement:

```text
positive supply ---- INx+  [ 0.1 ohm HAT shunt ]  INx- ---- DUT positive
negative supply ------------------------------------------------ DUT ground
       |                                                           |
       +---------------- HAT GND / Raspberry Pi GND ----------------+
```

- Follow the `INx+`, `INx-`, and `GND` silkscreen; do not rely on connector
  position alone.
- `INx+` is the source side. `INx-` is the DUT/load side. Reversing them changes
  the current sign.
- The dashboard's bus voltage is measured at `INx-` relative to HAT ground.
- Never connect the monitored 12 V rail to the Raspberry Pi 3.3 V or 5 V pins.
- A stock channel uses a 0.1 ohm shunt. Keep the dashboard shunt setting at
  `0.1` unless the HAT has been physically modified.

## External INA228 module

The lab uses this
[GODIYMODULES INA228 module](https://www.amazon.com/dp/B0GX5HM4X3).
It has an onboard `R002` (2 milliohm) shunt and is expected at I2C address
`0x44`, outside the HAT's `0x40`-`0x43` range. Do not add another shunt.

### Logic connection

The five-pin header is labeled in this order on the module:

| Module header | Raspberry Pi connection |
|---|---|
| `VCC` / `VS` | Physical pin 1 or 17 (3.3 V) |
| `SDA` | Physical pin 3 (GPIO2 / SDA1) |
| `SCL` | Physical pin 5 (GPIO3 / SCL1) |
| `ALE` / `ALERT` | Optional; leave disconnected unless explicitly used |
| `GND` | Physical pin 6, 9, or 14 |

### Current and bus-voltage connection

Looking at the component side with the five-pin header on the right, the orange
three-position screw terminal is labeled from top to bottom:

```text
top       VIN-   -> DUT positive (load side of onboard shunt)
middle    VBUS   -> jumper/sense wire to VIN- / DUT positive
bottom    VIN+   -> positive 12 V supply (source side of onboard shunt)
```

The complete high-side connection is:

```text
12 V supply + -> VIN+ -> [ onboard R002 shunt ] -> VIN- -> DUT positive
                                                   |
                                                   +---- VBUS

12 V supply - ---------------- module GND -------------- DUT ground
                                  |
                              Raspberry Pi GND
```

- Route the DUT load current through the `VIN+` and `VIN-` screw terminals.
- `VBUS` is a voltage-sense input, not a load-current terminal. Jumper it to
  `VIN-` to report the voltage delivered to the DUT.
- The onboard shunt is already 2 milliohms, matching the dashboard setting
  `0.002`. An incorrect shunt setting scales reported current directly.
- Follow the PCB labels before inserting wires. Product variants can change
  connector orientation even when the circuit is equivalent.

The dashboard currently uses legacy `--ina226-*` option names for settings
shared by external INA226 and INA228 monitors. For the lab INA228, the important
values are:

```text
--external-monitor-address 0x44
--ina226-shunt-ohms 0.002
--ina228-adc-range-mv 40.96
```

## Pre-power checklist

1. Power is off while wiring.
2. `IN+` is on the source side and `IN-` is on the DUT side.
3. Monitor ground, supply negative, and DUT ground have continuity.
4. The monitored rail is not connected to a Raspberry Pi power pin.
5. The INA228 is powered from 3.3 V, uses address `0x44`, and has `VBUS`
   jumpered to the DUT side (`VIN-`).
6. Check for shorts with a meter before applying power.
7. After applying power, confirm the bus voltage before trusting current or
   power. A nominal 12 V input should read close to 12 V, not approximately 1 V.

Check the live readings from the Raspberry Pi:

```bash
curl http://127.0.0.1:8080/api/ina219
curl http://127.0.0.1:8080/api/external-monitor
```

For device details, see the
[Waveshare Current/Power Monitor HAT documentation](https://www.waveshare.com/wiki/Current/Power_Monitor_HAT)
and the [TI INA228 data sheet](https://www.ti.com/lit/ds/symlink/ina228.pdf).
