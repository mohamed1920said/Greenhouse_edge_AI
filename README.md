# Edge-AI Smart Greenhouse

This repository is an experimental greenhouse monitoring and control project built around an ESP32-class controller. It combines environmental sensors, relay-driven actuators, MQTT/Node-RED connectivity, over-the-air updates, persistent setpoints, and an Edge Impulse anomaly model. It also contains the project's firmware history and KiCad hardware designs.

The project is a prototype, not a finished safety controller. The firmware and PCB directories contain several generations with different assumptions. Start with `code section/final/final.ino` for the most integrated implementation, then verify it against the exact PCB and board you intend to use before compiling or wiring anything.

## System overview

```text
DHT22 + BH1750 + soil + flow + AC-voltage + DS1302
                         |
                         v
                    ESP32 tasks
              sensors / control / AI / MQTT
                 |                     |
                 v                     v
      heater, fan, pump,          MQTT broker
      humidifier, light           and Node-RED
```

The current `final.ino` uses FreeRTOS tasks to:

- sample temperature, humidity, light, soil moisture, water flow, AC voltage, and RTC time every two seconds;
- operate heater, fan, irrigation pump, humidifier, and grow light in automatic or manual mode;
- run an Edge Impulse anomaly classifier every five seconds and add rule-based fault diagnoses;
- publish state as JSON and accept JSON commands through MQTT every two seconds; and
- service Arduino OTA updates from the main loop.

Setpoints are stored in ESP32 Preferences/NVS and survive a restart.

## Repository map

| Path | Contents |
| --- | --- |
| `code section/final/final.ino` | Integrated firmware and the recommended starting point for review. |
| `code section/V0` through `V6`, and `V8` through `V11` | Historical firmware iterations. `V7` is not present; these are alternatives, not modules to compile together. |
| `code section/TEST PCB/CODE/CODE.ino` | PCB-oriented test firmware. |
| `code section/logger/` | Python serial logger and an example CSV log. |
| `code section/LIBRARY/GreenhouseDashboard .zip` | Archived dashboard-related project files. |
| `ia library/ei-mohamed_said-project-1-arduino-1.0.11.zip` | Exported Edge Impulse Arduino inference library used by `final.ino`. |
| `LIBRARY/` | Archived SimpleTimer and DHT library packages used during earlier iterations. |
| `NODE RED/flows.json` | Node-RED flow export. Review broker and topic settings before importing. |
| `PCB/central/` | KiCad central-board design and backups. |
| `PCB/central/external/` | KiCad external-board design, a nested `pfe` design, and backups. |
| `iot-dashboard`, `PCB/central/.history`, `PCB/central/external/.history` | Gitlink entries whose referenced repositories are not configured in this repository. |

Do not assume that a pin map or sensor type from an older `V*` directory matches `final.ino`; variants in this repository use different hardware and dependencies.

The three Gitlink entries above are stored like submodules, but there is no `.gitmodules` file with their source URLs. A normal clone therefore cannot retrieve their contents with `git submodule update`; the original repository locations would need to be identified and configured first.

## Current firmware pin map

The following table describes only `code section/final/final.ino`:

| Function | GPIO | Notes |
| --- | ---: | --- |
| BH1750 SDA / SCL | 8 / 9 | I2C address `0x23`. |
| DHT22 data | 4 | `DHTTYPE` is `DHT22`. |
| Soil-moisture analog input | 14 | Mapped from raw dry `3200` and wet `1100` to 0-100%; recalibrate for the actual probe. |
| Water-flow pulse input | 10 | Interrupt input with pull-up; conversion assumes 7.5 pulses per flow-rate unit. |
| Heater relay | 5 | Active low in the helper functions. |
| Fan relay | 6 | Active low. |
| Irrigation-pump relay | 7 | Active low. |
| Humidifier output | 15 | Active high, unlike the other actuator relays. |
| Light relay | 16 | Active low. |
| Red alarm LED | 42 | Driven high during configured lockout conditions. |
| AC-voltage analog input | 3 | Uses a firmware calibration factor of `237.0`; requires a safe isolated sensing circuit. |
| DS1302 CLK / DAT / RST | 11 / 12 / 13 | Three-wire RTC connection. |

These GPIO numbers are board-specific. Select an ESP32 variant that exposes them, check boot/strapping and ADC restrictions, and compare this table with the selected KiCad revision before connecting hardware.

## Firmware dependencies

Use an Arduino-compatible ESP32 development environment and install the libraries required by `final.ino`:

- DHT sensor library;
- BH1750;
- PubSubClient;
- ArduinoJson;
- a DS1302-compatible RTC library providing `ThreeWire.h` and `RtcDS1302.h`;
- the Edge Impulse Arduino library from `ia library/ei-mohamed_said-project-1-arduino-1.0.11.zip`; and
- the Wi-Fi, Preferences, ArduinoOTA, Wire, and FreeRTOS support supplied by the chosen ESP32 Arduino core.

Library and ESP32-core versions are not pinned. Record known-good versions after the first successful build.

## Configuration and first run

1. Review the schematic and choose the firmware/PCB revision that matches the assembled hardware.
2. Install the dependencies and import the Edge Impulse ZIP as an Arduino library.
3. In `final.ino`, replace `WIFI_SSID`, `WIFI_PASS`, `MQTT_SERVER`, and `MQTT_PORT` with values for the deployment network. Do not publish real credentials in source control.
4. Verify every GPIO, relay active level, soil calibration point, flow-sensor factor, and voltage-sensor calibration with actuators disconnected.
5. Start an MQTT broker reachable from the controller. The checked-in defaults point to `192.168.4.1:1883` without authentication or TLS.
6. Optionally import `NODE RED/flows.json`, then inspect all nodes and broker settings before deploying the flow. The exported flow targets HiveMQ Cloud over TLS on port 8883, while `final.ino` targets `192.168.4.1:1883` without TLS or authentication; the two configurations will not interoperate unchanged.
7. Build and upload `code section/final/final.ino`, open a serial monitor at `115200`, and validate sensor readings.
8. Test one low-voltage actuator channel at a time. Only connect pumps, heaters, or mains-powered equipment after the logic and fail-safe states have been independently verified.

`connectWiFi()` blocks until the configured network becomes available. A wrong SSID/password therefore prevents the remaining setup from completing.

## MQTT interface

The device uses these topics:

| Direction | Topic | Purpose |
| --- | --- | --- |
| Device to broker | `greenhouse/data` | Sensor values, actuator state, anomaly result, mode, setpoints, and manual commands. |
| Broker to device | `greenhouse/commands` | Mode, setpoint, and actuator-command changes. |

Example automatic-mode configuration:

```json
{
  "mode": "AUTO",
  "temp_low": 18,
  "temp_high": 26,
  "hum_low": 50,
  "lux_low": 50,
  "moist_low": 30
}
```

Example manual command:

```json
{
  "mode": "MANUAL",
  "heater": false,
  "fan": true,
  "pump": false,
  "humid": false,
  "light": true
}
```

Telemetry includes `temp`, `humid`, `lux`, `moist`, `water_flow`, `mains_voltage`, the five `*_state` fields, `anomaly_score`, `diagnosis`, `mode`, `hour`, the current setpoints, and the five `cmd_*` fields. Invalid DHT readings are published as JSON `null`.

## Automatic control and diagnostics

The checked-in defaults are 18-26 deg C, 50% minimum humidity, 50 lux minimum light, and 30% minimum soil moisture. In automatic mode:

- the heater runs below the low-temperature setpoint and the fan above the high-temperature setpoint;
- the humidifier runs below the humidity setpoint;
- the pump runs below the soil-moisture setpoint; and
- the light uses a 150-lux off hysteresis above its on threshold.

The AI input repeats temperature, humidity, soil moisture, light, and water-flow values across the model frame. Rule-based diagnoses take priority for invalid DHT data, pump-without-flow, flow-without-pump, low temperature while heating, and low humidity while humidifying. An anomaly score above `3.0` otherwise produces a warning.

The controller switches off selected loads and lights the red LED when measured AC voltage is between 10 V and 190 V, or when the diagnosis contains `CRITICAL`. This logic is an application safeguard, not certified electrical protection.

## Known limitations

- Wi-Fi credentials and broker details are hard-coded. MQTT has no authentication or TLS, and OTA has no configured password.
- The code reads the RTC hour but currently sets `isDaytime = true`; automatic lighting is therefore not actually limited to the stated day period.
- The AC-voltage calculation assumes a particular 240 V sensor calibration. It must not be connected directly to mains, and its values are not reliable on different hardware without calibration.
- Brownout lockout is applied only when the measured voltage is above 10 V and below 190 V. A disconnected or failed sensing circuit that reports 0 V bypasses this check rather than forcing a safe state.
- On a DHT failure, automatic temperature/humidity branches skip new commands, so prior fan or humidifier states can remain latched. The resulting `CRITICAL` path explicitly switches off only the heater and pump; it does not force every actuator off.
- Soil and flow conversions use fixed calibration constants. Edge Impulse results are meaningful only if sensor order, units, preprocessing, and library export match the model's training data.
- Relay polarity is mixed: the humidifier is active high while the other actuator helpers are active low.
- Historical firmware, archived libraries, Node-RED flow, and PCB backups can conflict with the final sketch. There is no single checked-in bill of materials or version manifest tying them together.
- Three Gitlink entries have no matching `.gitmodules` configuration, so the dashboard and history content they reference is not reproducible from this repository alone.
- Control and network loops have not been demonstrated here as fault tolerant. Network loss, MCU reset, stuck relays, sensor disconnection, dry running, and watchdog behavior require explicit tests.

## Safety

Heaters, pumps, humidifiers, grow lights, relays, water, and mains sensing create fire, shock, flooding, and equipment hazards. Use isolated and correctly rated power supplies, relay/driver modules, fuses, grounding, enclosures, strain relief, emergency shutoff, and independent temperature/water-level protection. Keep low-voltage electronics separated from mains and wet areas. Test with safe dummy loads first, and do not operate this prototype unattended or use it as the only protection for plants, property, or people.
