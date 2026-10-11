#pragma once
#include "config.h"
#include "bms_engine.h"
#include "fault_fsm.h"

// Task 6: risk score, severity, and plain-English maintenance suggestions.
// Severity colors match the Task 4 states (also used on the LCD page and Blynk).
static const char* const STATE_COLOR[ST_COUNT] = {"#2ECC71", "#F1C40F", "#E67E22", "#E74C3C"};
static const char* const STATE_ICON[ST_COUNT]  = {"OK", "WARN", "FAILSAFE", "SHUTDOWN"};

class Analytics {
 public:
  static const uint32_t WINDOW_MS = 300000;   // fault-frequency window: 5 minutes

  void noteFault(uint32_t now) {
    _ts[_head] = now;
    _head = (_head + 1) % 8;
    if (_n < 8) _n++;
  }
  uint8_t faultsInWindow(uint32_t now) const {
    uint8_t c = 0;
    for (uint8_t i = 0; i < _n; i++) if (now - _ts[i] <= WINDOW_MS) c++;
    return c;
  }

  // Composite risk 0..100 from imbalance trend, fault frequency and SoC.
  float risk(const BmsSnapshot& s, SysState st, uint8_t recentFaults) const {
    float imbF = 0.6f * clamp01(s.imbalance / (2.0f * s.threshold))
               + 0.4f * clamp01(s.trend / 0.002f);
    float fltF = clamp01(recentFaults / 5.0f);
    float socF = s.soc < 20.0f ? (20.0f - s.soc) / 20.0f
               : (s.soc > 95.0f ? 0.3f * (s.soc - 95.0f) / 5.0f : 0.0f);
    float r = 100.0f * (0.40f * imbF + 0.35f * fltF + 0.25f * socF);
    static const float floorByState[ST_COUNT] = {0, 25, 70, 100};  // state sets a minimum
    return fmaxf(r, floorByState[st]);
  }

  static const char* level(float risk) { return risk < 30 ? "LOW" : (risk < 60 ? "MEDIUM" : "HIGH"); }

  // Human-readable advice built from live numbers.
  void suggest(char* out, size_t n, const BmsSnapshot& s, SysState st,
               FaultSource src, uint8_t recent) const {
    if (st == ST_SHUTDOWN)
      snprintf(out, n, "SHUTDOWN (%s fault). Isolate pack, inspect, then operator reset.", SRC_NAME[src]);
    else if (st == ST_FAILSAFE)
      snprintf(out, n, "FAILSAFE (%s). Load cut. Verify sensors and relay; recovery is checked.", SRC_NAME[src]);
    else if (s.soc < 15.0f)
      snprintf(out, n, "SoC %.0f%% critically low. Reduce load or start charging.", s.soc);
    else if (s.overThreshold && s.trendDir == TREND_RISING)
      snprintf(out, n, "Cell %u weak, imbalance %.0fmV rising (limit %.0f). Schedule balancing.",
               s.weakest + 1, s.imbalance * 1000, s.threshold * 1000);
    else if (s.overThreshold)
      snprintf(out, n, "Imbalance %.0fmV above limit %.0fmV. Check cell %u.",
               s.imbalance * 1000, s.threshold * 1000, s.weakest + 1);
    else if (recent >= 3)
      snprintf(out, n, "%u faults in 5 min. Inspect wiring, sensors and relay.", recent);
    else if (st == ST_DEGRADED)
      snprintf(out, n, "Degraded (%s). Monitor closely; recovery in progress.", SRC_NAME[src]);
    else
      snprintf(out, n, "Pack healthy. SoC %.0f%%, imbalance %.0fmV. No action needed.",
               s.soc, s.imbalance * 1000);
  }

 private:
  static float clamp01(float x) { return fminf(fmaxf(x, 0.0f), 1.0f); }
  uint32_t _ts[8] = {0};
  uint8_t _head = 0, _n = 0;
};
