#pragma once
// Fill these from your Blynk template/device (see README.md).
#pragma once
#include "secrets.h"
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "config.h"
#include "bms_engine.h"
#include "fault_fsm.h"
#include "analytics.h"
// Wokwi's built-in Wi-Fi. For real hardware, put your own network here.
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""
