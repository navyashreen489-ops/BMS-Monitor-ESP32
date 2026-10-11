#pragma once
#include <stdarg.h>
#include <LiquidCrystal_I2C.h>
#include "config.h"
#include "fault_fsm.h"

// Task 3: flicker-free LCD. We never call clear() after startup. A shadow copy of
// the screen is compared with the new text, and only changed characters are written.

struct LcdData {
  float soc = 0, imbMv = 0;
  uint8_t weak = 0, strong = 0;
  SysState st = ST_NORMAL;
  bool relay = false;
  uint16_t faults = 0;
  bool online = false;
  int rssi = 0;
  uint8_t qDepth = 0;
  const char* mode = "OFFLINE";
};

class LcdEngine {
 public:
  LcdEngine() : _lcd(0x27, 16, 2) {}

  void begin() {
    _lcd.init();
    _lcd.backlight();
    _lcd.clear();                       // the only clear, at startup
    for (uint8_t r = 0; r < 2; r++)
      for (uint8_t c = 0; c < 16; c++) _sh[r][c] = ' ';
  }

  // Called from the state-machine transition hook: fault screen takes over at once.
  void latchFault(FaultSource src, uint8_t id) { _latched = true; _src = src; _id = id; _ack = false; }
  // Operator acknowledges (reset button). Screen clears once the state is below FAILSAFE.
  void acknowledge() { _ack = true; }

  void update(uint32_t now, const LcdData& d) {
    if (now - _lastDraw < LCD_REFRESH_MS) return;     // non-blocking refresh timer
    _lastDraw = now;

    if (_latched && _ack && d.st < ST_FAILSAFE) { _latched = false; _ack = false; }

    char l0[17], l1[17];
    if (_latched) {
      fmt(l0, "FAULT %-9s", STATE_NAME[d.st]);
      fmt(l1, "%s #%u ack=RST", SRC_NAME[_src], (unsigned)_id);
    } else {
      switch ((now / PAGE_MS) % 3) {                  // auto-rotating pages
        case 0:
          fmt(l0, "SoC%3.0f%% Imb%3.0fmV", d.soc, d.imbMv);
          fmt(l1, "Weak C%u Strg C%u", d.weak + 1, d.strong + 1);
          break;
        case 1:
          fmt(l0, "State:%-9s", STATE_NAME[d.st]);
          fmt(l1, "Relay:%s Flt:%u", d.relay ? "ON " : "OFF", (unsigned)d.faults);
          break;
        default:
          if (d.online) fmt(l0, "WiFi%4ddBm Q:%2u", d.rssi, (unsigned)d.qDepth);
          else          fmt(l0, "WiFi DOWN  Q:%2u", (unsigned)d.qDepth);
          fmt(l1, "Data:%s", d.mode);
      }
    }
    draw(0, l0);
    draw(1, l1);
  }

 private:
  static void fmt(char* dst, const char* f, ...) {
    va_list a; va_start(a, f);
    vsnprintf(dst, 17, f, a);
    va_end(a);
    size_t n = strlen(dst);
    while (n < 16) dst[n++] = ' ';
    dst[16] = 0;
  }

  // Write only the characters that differ from what is already on the screen.
  void draw(uint8_t row, const char* t) {
    int cursor = -1;
    for (uint8_t c = 0; c < 16; c++) {
      if (_sh[row][c] != t[c]) {
        if (cursor != (int)c) _lcd.setCursor(c, row);
        _lcd.write(t[c]);
        _sh[row][c] = t[c];
        cursor = c + 1;
      }
    }
  }

  LiquidCrystal_I2C _lcd;
  char _sh[2][16];
  uint32_t _lastDraw = 0;
  bool _latched = false, _ack = false;
  FaultSource _src = SRC_NONE;
  uint8_t _id = 0;
};
