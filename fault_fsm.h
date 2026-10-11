#pragma once
#include "config.h"

// Task 4: deterministic fault state machine.

enum SysState : uint8_t { ST_NORMAL, ST_DEGRADED, ST_FAILSAFE, ST_SHUTDOWN, ST_COUNT };
enum FaultSource : uint8_t { SRC_NONE, SRC_CELL, SRC_RELAY, SRC_COMM, SRC_ADC, SRC_COUNT };
enum Severity : uint8_t { SEV_NONE, SEV_MINOR, SEV_MAJOR, SEV_CRITICAL };

static const char* const STATE_NAME[ST_COUNT] = {"NORMAL", "DEGRADED", "FAILSAFE", "SHUTDOWN"};
static const char* const SRC_NAME[SRC_COUNT] = {"NONE", "CELL", "RELAY", "COMM", "ADC"};

// Transition table [from][to]. No direct FAILSAFE->NORMAL (must verify via DEGRADED).
// SHUTDOWN can only be left toward FAILSAFE, and only by operator reset.
static const bool TRANSITION_OK[ST_COUNT][ST_COUNT] = {
  /* NORMAL   */ {0, 1, 1, 1},
  /* DEGRADED */ {1, 0, 1, 1},
  /* FAILSAFE */ {0, 1, 0, 1},
  /* SHUTDOWN */ {0, 0, 1, 0},
};

struct TransitionLog {
  uint32_t t;
  SysState from, to;
  FaultSource src;
  uint8_t faultId;
};

class FaultFsm {
 public:
  // Optional hook: Tasks 3 and 5/6 subscribe here so nothing misses a transition.
  void (*onTransition)(const TransitionLog&) = nullptr;

  void reportFault(FaultSource s, uint8_t id, Severity sev) {
    if (sev >= _f[s].sev) { _f[s].sev = sev; _f[s].id = id; }
  }
  void clearFault(FaultSource s) { _f[s].sev = SEV_NONE; }
  void operatorReset() { _resetRequested = true; }

  void update(uint32_t now) {
    Severity w = worst();
    SysState target = w == SEV_CRITICAL ? ST_SHUTDOWN
                    : w == SEV_MAJOR    ? ST_FAILSAFE
                    : w == SEV_MINOR    ? ST_DEGRADED : ST_NORMAL;

    if (_state == ST_SHUTDOWN) {
      if (_resetRequested && w < SEV_CRITICAL) go(ST_FAILSAFE, now);
      _resetRequested = false;
      return;
    }
    if (target > _state) {            // escalate immediately
      go(target, now);
      _cleanSince = 0;
    } else if (target < _state) {     // recover one step at a time, after a hold
      if (_cleanSince == 0) _cleanSince = now;
      uint32_t hold = (_state == ST_FAILSAFE) ? VERIFY_MS : RECOVER_MS;
      if (now - _cleanSince >= hold) {
        go((SysState)(_state - 1), now);
        _cleanSince = 0;
      }
    } else {
      _cleanSince = 0;
    }
  }

  SysState state() const { return _state; }
  bool verifying() const { return _state == ST_FAILSAFE && _cleanSince != 0; }
  uint32_t faultCount() const { return _faultCount; }

 private:
  struct Active { Severity sev = SEV_NONE; uint8_t id = 0; } _f[SRC_COUNT];
  SysState _state = ST_NORMAL;
  uint32_t _cleanSince = 0;
  uint32_t _faultCount = 0;
  bool _resetRequested = false;

  Severity worst() const {
    Severity w = SEV_NONE;
    for (uint8_t i = 1; i < SRC_COUNT; i++) if (_f[i].sev > w) w = _f[i].sev;
    return w;
  }
  FaultSource dominantSource() const {
    FaultSource d = SRC_NONE; Severity w = SEV_NONE;
    for (uint8_t i = 1; i < SRC_COUNT; i++) if (_f[i].sev > w) { w = _f[i].sev; d = (FaultSource)i; }
    return d;
  }
  void go(SysState to, uint32_t now) {
    if (!TRANSITION_OK[_state][to]) return;   // table is the single source of truth
    FaultSource src = dominantSource();
    TransitionLog e = {now, _state, to, src, _f[src].id};
    if (to > _state) _faultCount++;
    _state = to;
    // Structured, machine-readable log line.
    Serial.printf("{\"t\":%lu,\"from\":\"%s\",\"to\":\"%s\",\"src\":\"%s\",\"fault\":%u}\n",
                  (unsigned long)e.t, STATE_NAME[e.from], STATE_NAME[e.to],
                  SRC_NAME[e.src], e.faultId);
    if (onTransition) onTransition(e);
  }
};

// ---- Detectors that feed reportFault() ----

// Flags a reading that has not moved for too long (frozen ADC).
class FrozenDetector {
 public:
  bool update(float v, uint32_t now, float eps = 0.0005f) {
    if (!_init || fabsf(v - _last) > eps) { _last = v; _since = now; _init = true; }
    return (now - _since) > ADC_FROZEN_MS;
  }
 private:
  float _last = 0; uint32_t _since = 0; bool _init = false;
};

// Flags commanded vs. measured relay state disagreeing for too long.
class RelayMismatch {
 public:
  bool update(bool commanded, bool feedback, uint32_t now) {
    if (commanded == feedback) { _active = false; return false; }
    if (!_active) { _active = true; _since = now; }
    return (now - _since) > RELAY_MISMATCH_MS;
  }
 private:
  bool _active = false; uint32_t _since = 0;
};
