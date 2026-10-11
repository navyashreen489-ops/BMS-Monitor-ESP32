# BMS-Monitor-ESP32
Battery Management System with fault state machine, protection relay, LCD engine, and Blynk IoT dashboard — ElevanceSkills Internship Project (ESP32, Arduino C++).
# BMS Monitor — ESP32

Battery Management System covering 6 tasks:
1. Modular BMS Engine (cell analysis, SoC, adaptive thresholds)
2. Non-blocking Protection Relay (hysteresis, debounce, noise filter)
3. Flicker-free LCD Engine (3 pages, fault screen)
4. Fault State Machine (NORMAL, DEGRADED, FAILSAFE, SHUTDOWN)
5. Event-driven Blynk Telemetry (offline queue, Wi-Fi state machine)
6. Analytics Dashboard (risk score, maintenance suggestions)

## How to run
- Open in Wokwi: https://wokwi.com/projects/477412197445050369
- Press Play. Set knobs to 75% before running.
- Add your Blynk token to secrets.h

 ## Fault injection demo
- Red button: inject frozen ADC fault
- Orange button: relay mismatch fault
- Yellow button: sensor spike (rejected by noise filter)
- Green button: operator reset / acknowledge
- Blue button: toggle Wi-Fi outage

## Hardware (simulated)
- ESP32 DevKit
- 4 potentiometers (battery cells)
- LCD 1602 I2C
- Relay module
- 5 push buttons (fault injection)

## Tools
ESP32, Arduino C++, Wokwi, Blynk IoT, GitHub

## Blynk dashboard
[[paste your blynk dashboard URL here](https://blynk.cloud/dashboard/775543/templates/1618357/dashboard)]
