#pragma once
#include <Arduino.h>

// ---- Task 1: scalable cell array (change only this line to scale 4 -> 16) ----
#define NUM_CELLS 4

// ADC pins for the cells (ESP32). For 16 cells use an analog multiplexer
// (e.g. CD74HC4067) and loop over channels instead of one pin per cell.
static const uint8_t CELL_PINS[NUM_CELLS] = {34, 35, 32, 33};

// ---- Cell voltage / SoC model (tune to your chemistry) ----
static const float V_EMPTY = 3.0f;
static const float V_FULL  = 4.2f;

// ---- Adaptive imbalance threshold ----
static const float BASE_THR_V   = 0.050f;  // threshold at mid SoC
static const float LOW_SOC_MULT = 2.0f;    // looser when battery is low
static const float HIGH_SOC_MULT = 0.8f;   // tighter when nearly full
static const float RATE_GAIN    = 50.0f;   // extra tolerance per V/s of discharge

// ---- Imbalance trend ----
#define TREND_WINDOW 8
static const float TREND_DEADBAND = 0.0005f;  // V per sample

// ---- Task 4: timing ----
static const uint32_t RECOVER_MS = 5000;   // clean time before DEGRADED -> NORMAL
static const uint32_t VERIFY_MS  = 8000;   // verification before FAILSAFE -> DEGRADED
static const uint32_t ADC_FROZEN_MS = 3000;
static const uint32_t RELAY_MISMATCH_MS = 300;

// ---- Simulation pins (ESP32 in Wokwi) ----
#define RELAY_PIN      26
#define BTN_FREEZE     18   // toggles "frozen ADC" on cell 1
#define BTN_RELAYFAULT 19   // toggles "relay feedback mismatch"
#define BTN_SPIKE      23   // injects a one-sample spike on cell 2
#define BTN_RESET       5   // operator reset / acknowledge + clears injected faults
#define BTN_OUTAGE      4   // toggles a simulated Wi-Fi outage

// Wokwi's ADC is perfectly clean, so we map the pot range to 2.8..4.2 V and add
// a few millivolts of jitter like a real ADC. A truly frozen value then stands out.
static const float V_SIM_MIN  = 2.8f;
static const float SIM_NOISE_V = 0.003f;

// ---- Task 2: protection relay ----
static const float UV_TRIP_V    = 3.20f;   // undervoltage trips below this...
static const float UV_RELEASE_V = 3.40f;   // ...and only releases above this (hysteresis)
static const uint32_t RELAY_DEBOUNCE_MS = 200;
static const uint32_t RELAY_RECOVERY_MS = 3000;
static const float SPIKE_V = 0.30f;        // jump size that counts as a suspicious step

// ---- Task 3: LCD ----
static const uint32_t LCD_REFRESH_MS = 250;
static const uint32_t PAGE_MS = 3000;

// ---- Sampling ----
static const uint32_t SAMPLE_MS = 500;

// ---- Task 5: telemetry ----
#define QUEUE_SIZE 20
static const uint32_t REPLAY_GAP_MS = 700;   // pacing so replay doesn't flood Blynk
static const uint32_t HEARTBEAT_MS  = 60000; // keep-alive when nothing changes
static const float CELL_DELTA_V = 0.02f;
static const float SOC_DELTA    = 1.0f;
static const float RISK_DELTA   = 5.0f;
static const uint32_t COMM_FAULT_MS = 15000; // offline this long => COMM fault (DEGRADED)
#define ENABLE_COMM_FAULT 1
#define TIP_LEN 96
