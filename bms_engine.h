#pragma once
#include "config.h"

// Task 1: reusable analysis module. Later tasks (relay, LCD, Blynk, analytics)
// read BmsSnapshot only; none of them recompute battery logic.

enum ImbTrend : int8_t { TREND_FALLING = -1, TREND_STABLE = 0, TREND_RISING = 1 };

struct BmsSnapshot {
  float volt[NUM_CELLS] = {0};
  uint8_t weakest = 0, strongest = 0;
  float imbalance = 0;       // volts, strongest - weakest
  float trend = 0;           // volts per sample (>0 = growing)
  ImbTrend trendDir = TREND_STABLE;
  float soc = 0;             // percent
  float dischargeRate = 0;   // volts per second (smoothed, >0 = discharging)
  float threshold = BASE_THR_V;
  bool overThreshold = false;
};

class BmsEngine {
 public:
  // Call once per sample with NUM_CELLS voltages.
  void update(const float* v, uint32_t now) {
    float sum = 0;
    uint8_t w = 0, s = 0;
    for (uint8_t i = 0; i < NUM_CELLS; i++) {
      _s.volt[i] = v[i];
      sum += v[i];
      if (v[i] < v[w]) w = i;
      if (v[i] > v[s]) s = i;
    }
    _s.weakest = w;
    _s.strongest = s;
    _s.imbalance = v[s] - v[w];

    float avg = sum / NUM_CELLS;
    float soc = (avg - V_EMPTY) / (V_FULL - V_EMPTY);
    _s.soc = 100.0f * fminf(fmaxf(soc, 0.0f), 1.0f);

    // Smoothed discharge rate (exponential moving average).
    if (_lastT != 0 && now > _lastT) {
      float dt = (now - _lastT) / 1000.0f;
      float inst = (_lastAvg - avg) / dt;
      _s.dischargeRate += 0.2f * (inst - _s.dischargeRate);
    }
    _lastT = now;
    _lastAvg = avg;

    // Trend from a fixed-size ring buffer (no heap allocation).
    _hist[_head] = _s.imbalance;
    _head = (_head + 1) % TREND_WINDOW;
    if (_count < TREND_WINDOW) _count++;
    if (_count == TREND_WINDOW) {
      float oldest = _hist[_head];  // after increment, head points at oldest
      _s.trend = (_s.imbalance - oldest) / (TREND_WINDOW - 1);
      _s.trendDir = _s.trend > TREND_DEADBAND ? TREND_RISING
                  : _s.trend < -TREND_DEADBAND ? TREND_FALLING : TREND_STABLE;
    }

    // Adaptive threshold: depends on SoC and discharge rate, not a fixed limit.
    float thr = BASE_THR_V;
    if (_s.soc < 20.0f) thr *= LOW_SOC_MULT;
    else if (_s.soc > 80.0f) thr *= HIGH_SOC_MULT;
    thr *= 1.0f + RATE_GAIN * fmaxf(_s.dischargeRate, 0.0f);
    _s.threshold = thr;
    _s.overThreshold = _s.imbalance > thr;
  }

  const BmsSnapshot& snapshot() const { return _s; }

 private:
  BmsSnapshot _s;
  float _hist[TREND_WINDOW] = {0};
  uint8_t _head = 0, _count = 0;
  uint32_t _lastT = 0;
  float _lastAvg = 0;
};
