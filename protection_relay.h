#pragma once
#include "config.h"
#include "fault_fsm.h"

// ---------- Task 2a: sensor validation (noise vs. real load change) ----------
// Median-of-3 window. A genuine step stays at its new level for the next
// samples, so it passes. A one-sample spike jumps away and comes straight back,
// so it is rejected and counted. Many spikes in a short time => noisy sensor.
class SensorValidator {
 public:
  float filter(float raw, uint32_t now) {
    if (raw < 2.0f || raw > 4.6f) { if (_rangeCount < 255) _rangeCount++; }
    else _rangeCount = 0;

    _h[0] = _h[1]; _h[1] = _h[2]; _h[2] = raw;
    if (_n < 3) { _n++; return raw; }

    bool spike = fabsf(_h[1] - _h[0]) > SPIKE_V &&
                 fabsf(_h[1] - _h[2]) > SPIKE_V &&
                 fabsf(_h[2] - _h[0]) < SPIKE_V;
    if (spike && _score < 10) _score++;
    if (now - _lastDecay >= 1000) { _lastDecay = now; if (_score) _score--; }
    return median3(_h[0], _h[1], _h[2]);
  }
  bool rangeBad() const { return _rangeCount >= 3; }  // out of range for 3 samples in a row
  bool noisy() const { return _score >= 3; }

 private:
  static float median3(float a, float b, float c) {
    return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
  }
  float _h[3] = {0, 0, 0};
  uint8_t _n = 0, _score = 0, _rangeCount = 0;
  uint32_t _lastDecay = 0;
};

// ---------- Task 2b: non-blocking protection relay ----------
// OPEN -> RECOVERY (timed wait) -> CLOSED. No delay() anywhere.
enum RelayState : uint8_t { RS_OPEN, RS_RECOVERY, RS_CLOSED };
static const char* const RELAY_NAME[3] = {"OPEN", "RECOVERY", "CLOSED"};

class ProtectionRelay {
 public:
  void begin(uint8_t pin) {
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);
  }

  // Undervoltage with hysteresis: trips below UV_TRIP_V, releases above UV_RELEASE_V.
  bool undervolt(float minCellV) {
    if (!_uv && minCellV < UV_TRIP_V) _uv = true;
    else if (_uv && minCellV > UV_RELEASE_V) _uv = false;
    return _uv;
  }

  void update(uint32_t now, SysState st) {
    bool healthy = (st == ST_NORMAL || st == ST_DEGRADED);
    switch (_rs) {
      case RS_CLOSED:
        if (!healthy) {                       // debounce: must stay unhealthy
          if (!_bad) { _bad = true; _badSince = now; }
          if (now - _badSince >= RELAY_DEBOUNCE_MS) {
            digitalWrite(_pin, LOW);
            change(RS_OPEN, now);
          }
        } else {
          _bad = false;
        }
        break;
      case RS_OPEN:
        if (healthy) change(RS_RECOVERY, now);
        break;
      case RS_RECOVERY:                       // timed recovery before closing again
        if (!healthy) change(RS_OPEN, now);
        else if (now - _t >= RELAY_RECOVERY_MS) {
          digitalWrite(_pin, HIGH);
          change(RS_CLOSED, now);
        }
        break;
    }
  }

  bool closed() const { return _rs == RS_CLOSED; }   // commanded state
  RelayState state() const { return _rs; }

 private:
  void change(RelayState n, uint32_t now) {
    Serial.printf("{\"t\":%lu,\"relay\":\"%s\",\"to\":\"%s\"}\n",
                  (unsigned long)now, RELAY_NAME[_rs], RELAY_NAME[n]);
    _rs = n; _t = now; _bad = false;
  }
  uint8_t _pin = 26;
  RelayState _rs = RS_OPEN;
  uint32_t _t = 0, _badSince = 0;
  bool _bad = false, _uv = false;
};
