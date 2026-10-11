 #pragma once
#define BLYNK_TEMPLATE_ID   "TMPL3qmvKcKhQ"
#define BLYNK_TEMPLATE_NAME "BMS Monitor"
#define BLYNK_AUTH_TOKEN    "PUT_YOUR_TOKEN_HERE"
#define WIFI_SSID "Wokwi-GUEST"
#define WIFI_PASS ""
#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include "config.h"
#include "bms_engine.h"
#include "fault_fsm.h"
#include "analytics.h"

// Task 5: event-driven telemetry. Data is sent only when something meaningful
// changes. While offline, events go into a fixed-size queue and are replayed in order.

enum BlynkPin : uint8_t {
  P_CELL0 = 0, P_WEAKSTR = 4, P_RELAY = 5, P_STATE = 6,
  P_RSSI = 7, P_QDEPTH = 8, P_MODE = 9, P_SOC = 10,
  P_IMB = 11, P_RISK = 12, P_TIP = 13, P_UPTIME = 14,
  P_FAULTS = 15, P_HEALTH = 16, P_LOG = 17
};

struct Packet {
  uint32_t t;
  float volt[NUM_CELLS];
  uint8_t weak, strong;
  SysState state;
  bool relay;
  float soc, imb, risk, health;
  uint16_t faults;
  bool hasTr;
  TransitionLog tr;
  char tip[TIP_LEN];
};

enum WifiState : uint8_t { W_IDLE, W_CONNECTING, W_UP, W_BACKOFF };

class Telemetry {
 public:
  bool simulateOutage = false;

  void begin() {
    invalidateCache();
    WiFi.mode(WIFI_STA);
    Blynk.config("PUT_YOUR_TOKEN_HERE", "blynk.cloud", 80);
  }

  void update(uint32_t now) {
    wifiFsm(now);
    if (_w == W_UP) {
      if (Blynk.connected()) {
        Blynk.run();
      } else if (now - _bT >= _bBackoff) {
        _bT = now;
        if (Blynk.connect(1500)) _bBackoff = 2000;
        else _bBackoff = (_bBackoff * 2 > 16000) ? 16000 : _bBackoff * 2;
      }
    }
    bool on = online();
    if (on) {
      if (!_wasOnline) { invalidateCache(); _lastRssi = 1000; _lastQ = 255; }
      _lastOnline = now;
      drain(now);
      health();
    }
    _wasOnline = on;
  }

  void consider(const Packet& p, uint32_t now) {
    bool sig = !_haveLast || p.hasTr || p.state != _last.state || p.relay != _last.relay ||
               fabsf(p.soc - _last.soc) >= SOC_DELTA ||
               fabsf(p.risk - _last.risk) >= RISK_DELTA ||
               (now - _lastEvent) >= HEARTBEAT_MS;
    for (uint8_t i = 0; i < NUM_CELLS && !sig; i++)
      if (fabsf(p.volt[i] - _last.volt[i]) >= CELL_DELTA_V) sig = true;
    if (!sig) return;
    _last = p; _haveLast = true; _lastEvent = now;
    if (online() && _count == 0) send(p, false);
    else push(p);
  }

  bool online() const { return _w == W_UP && Blynk.connected(); }
  uint8_t queueDepth() const { return _count; }
  uint16_t dropped() const { return _dropped; }
  int rssi() const { return WiFi.RSSI(); }
  uint32_t commLostMs(uint32_t now) const { return online() ? 0 : now - _lastOnline; }
  const char* modeText() const { return !online() ? "OFFLINE" : (_count > 0 ? "QUEUED" : "LIVE"); }

 private:
  void wifiFsm(uint32_t now) {
    switch (_w) {
      case W_IDLE:
        if (!simulateOutage) { WiFi.begin(WIFI_SSID, WIFI_PASS, 6); _w = W_CONNECTING; _wT = now; }
        break;
      case W_CONNECTING:
        if (simulateOutage) { WiFi.disconnect(); _w = W_BACKOFF; _wT = now; }
        else if (WiFi.status() == WL_CONNECTED) { _w = W_UP; _backoff = 1000; _bT = 0; _bBackoff = 0; }
        else if (now - _wT > 10000) { WiFi.disconnect(); _w = W_BACKOFF; _wT = now; }
        break;
      case W_UP:
        if (simulateOutage || WiFi.status() != WL_CONNECTED) {
          Blynk.disconnect(); WiFi.disconnect();
          _w = W_BACKOFF; _wT = now;
        }
        break;
      case W_BACKOFF:
        if (!simulateOutage && now - _wT >= _backoff) {
          _backoff = (_backoff * 2 > 16000) ? 16000 : _backoff * 2;
          _w = W_IDLE;
        }
        break;
    }
  }

  void push(const Packet& p) {
    if (_count == QUEUE_SIZE) { _head = (_head + 1) % QUEUE_SIZE; _count--; _dropped++; }
    _q[(_head + _count) % QUEUE_SIZE] = p; _count++;
  }
  void drain(uint32_t now) {
    if (_count == 0 || now - _lastSend < REPLAY_GAP_MS) return;
    Packet p = _q[_head]; _head = (_head + 1) % QUEUE_SIZE; _count--;
    send(p, true);
  }
  void health() {
    int r = WiFi.RSSI();
    if (abs(r - _lastRssi) >= 5) { Blynk.virtualWrite(P_RSSI, r); _lastRssi = r; }
    if (_count != _lastQ) { Blynk.virtualWrite(P_QDEPTH, (int)_count); _lastQ = _count; }
  }
  void wf(uint8_t pin, float v) {
    if (_lf[pin] != _lf[pin] || fabsf(_lf[pin] - v) > 0.001f) { Blynk.virtualWrite(pin, v); _lf[pin] = v; }
  }
  void ws(uint8_t pin, const char* s) {
    if (_ls[pin] != s) { Blynk.virtualWrite(pin, s); _ls[pin] = s; }
  }
  void invalidateCache() {
    for (uint8_t i = 0; i < 18; i++) { _lf[i] = NAN; _ls[i] = ""; }
    _shownState = 255;
  }
  void send(const Packet& p, bool queued) {
    for (uint8_t i = 0; i < NUM_CELLS && i < 4; i++) wf(P_CELL0 + i, p.volt[i]);
    char b[56];
    snprintf(b, sizeof b, "Weak C%u %.2fV | Strong C%u %.2fV",
             p.weak+1, p.volt[p.weak], p.strong+1, p.volt[p.strong]);
    ws(P_WEAKSTR, b);
    wf(P_RELAY, p.relay ? 1 : 0);
    if (p.state != _shownState) {
      Blynk.setProperty(P_STATE, "color", STATE_COLOR[p.state]);
      _shownState = p.state;
    }
    ws(P_STATE, STATE_NAME[p.state]);
    ws(P_MODE, (queued && _count > 0) ? "QUEUED (replay)" : "LIVE");
    Blynk.virtualWrite(P_STATE, STATE_NAME[p.state]);
    wf(P_RISK, p.risk); wf(P_HEALTH, p.health);
   Blynk.virtualWrite(P_STATE, STATE_NAME[p.state]);
    wf(P_UPTIME, p.t / 1000.0f); wf(P_FAULTS, p.faults);
    if (p.hasTr) {
      char line[110];
      snprintf(line, sizeof line, "[%lus] %s->%s src=%s fault=%u%s\n",
               (unsigned long)(p.tr.t/1000), STATE_NAME[p.tr.from], STATE_NAME[p.tr.to],
               SRC_NAME[p.tr.src], (unsigned)p.tr.faultId, queued ? " (queued)" : "");
      Blynk.virtualWrite(P_LOG, line);
    }
    _lastSend = millis();
  }

  WifiState _w = W_IDLE;
  uint32_t _wT = 0, _backoff = 1000, _bT = 0, _bBackoff = 0;
  uint32_t _lastOnline = 0, _lastSend = 0, _lastEvent = 0;
  bool _wasOnline = false, _haveLast = false;
  Packet _last, _q[QUEUE_SIZE];
  uint8_t _head = 0, _count = 0;
  uint16_t _dropped = 0;
  int _lastRssi = 1000, _lastQ = 255;
  float _lf[18];
  String _ls[18];
  uint8_t _shownState = 255;
};
