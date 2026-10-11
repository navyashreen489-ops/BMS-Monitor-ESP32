// BMS project: Tasks 1-6 combined.
// Task 1 bms_engine.h | Task 2 protection_relay.h | Task 3 lcd_engine.h
// Task 4 fault_fsm.h  | Task 5 telemetry.h        | Task 6 analytics.h
#include "config.h"
#include "bms_engine.h"
#include "fault_fsm.h"
#include "protection_relay.h"
#include "analytics.h"
#include "lcd_engine.h"
#include "telemetry.h"

BmsEngine bms;
FaultFsm fsm;
ProtectionRelay relay;
Analytics ana;
LcdEngine lcd;
Telemetry tel;
FrozenDetector frozen[NUM_CELLS];
SensorValidator validator[NUM_CELLS];
RelayMismatch mismatch;

// Fault-injection switches (buttons in the simulator)
bool injFreeze = false, injRelay = false, injSpike = false, outageOn = false;
float frozenValue = 3.5f, lastRaw0 = 3.5f;

FaultSource lastSrc = SRC_NONE;
TransitionLog pendingTr;
bool havePendingTr = false;
uint32_t lastSample = 0;

struct Button {
  uint8_t pin; bool stable; uint32_t t;
  Button(uint8_t p) : pin(p), stable(true), t(0) {}
  bool fell(uint32_t now) {
    bool r = digitalRead(pin);
    if (r != stable && now - t > 40) { t = now; stable = r; return r == LOW; }
    return false;
  }
};
Button bFreeze(BTN_FREEZE), bRelay(BTN_RELAYFAULT), bSpike(BTN_SPIKE), bReset(BTN_RESET), bOutage(BTN_OUTAGE);

// Every state transition lands here: LCD fault screen, analytics, and Blynk packet.
void onTr(const TransitionLog& e) {
  pendingTr = e; havePendingTr = true;
  if (e.to > e.from) ana.noteFault(e.t);
  if (e.to >= ST_FAILSAFE) lcd.latchFault(e.src, e.faultId);
  if (e.src != SRC_NONE) lastSrc = e.src;
}

void handleButtons(uint32_t now) {
  if (bFreeze.fell(now))  { injFreeze = !injFreeze; frozenValue = lastRaw0; Serial.printf("[inject] frozen ADC = %d\n", injFreeze); }
  if (bRelay.fell(now))   { injRelay = !injRelay;   Serial.printf("[inject] relay mismatch = %d\n", injRelay); }
  if (bSpike.fell(now))   { injSpike = true;        Serial.println("[inject] sensor spike"); }
  if (bOutage.fell(now))  { outageOn = !outageOn;   Serial.printf("[inject] Wi-Fi outage = %d\n", outageOn); }
  if (bReset.fell(now))   { injFreeze = false; injRelay = false; fsm.operatorReset(); lcd.acknowledge(); Serial.println("[operator] reset / acknowledge"); }
}

void sampleAndAnalyze(uint32_t now) {
  float v[NUM_CELLS];
  bool frozenAny = false, rangeBad = false, noisy = false;

  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    float x = V_SIM_MIN + (analogRead(CELL_PINS[i]) / 4095.0f) * (V_FULL - V_SIM_MIN);
    x += ((int)random(-100, 101) / 100.0f) * SIM_NOISE_V;      // imitate real ADC jitter
    if (i == 0) { if (!injFreeze) lastRaw0 = x; else x = frozenValue; }
    if (i == 1 && injSpike) { x += 1.2f; injSpike = false; }
    frozenAny |= frozen[i].update(x, now);
    v[i] = validator[i].filter(x, now);
    rangeBad |= validator[i].rangeBad();
    noisy    |= validator[i].noisy();
  }

  bms.update(v, now);
  const BmsSnapshot& s = bms.snapshot();

  // ADC faults
  fsm.clearFault(SRC_ADC);
  if (rangeBad)       fsm.reportFault(SRC_ADC, 7, SEV_MAJOR);
  else if (frozenAny) fsm.reportFault(SRC_ADC, 1, SEV_MAJOR);
  else if (noisy)     fsm.reportFault(SRC_ADC, 6, SEV_MINOR);

  // Cell faults
  bool uv = relay.undervolt(v[s.weakest]);
  fsm.clearFault(SRC_CELL);
  if (s.soc < 5.0f)                      fsm.reportFault(SRC_CELL, 3, SEV_CRITICAL);
  else if (uv)                           fsm.reportFault(SRC_CELL, 5, SEV_MAJOR);
  else if (s.imbalance > 2 * s.threshold) fsm.reportFault(SRC_CELL, 4, SEV_MAJOR);
  else if (s.overThreshold)              fsm.reportFault(SRC_CELL, 2, SEV_MINOR);
}

void publish(uint32_t now) {
  const BmsSnapshot& s = bms.snapshot();
  Packet p;
  p.t = now;
  for (uint8_t i = 0; i < NUM_CELLS; i++) p.volt[i] = s.volt[i];
  p.weak = s.weakest; p.strong = s.strongest;
  p.state = fsm.state();
  p.relay = relay.closed();
  p.soc = s.soc; p.imb = s.imbalance;
  uint8_t recent = ana.faultsInWindow(now);
  p.risk = ana.risk(s, p.state, recent);
  p.health = 100.0f - p.risk;
  p.faults = (uint16_t)fsm.faultCount();
  p.hasTr = havePendingTr;
  if (havePendingTr) { p.tr = pendingTr; havePendingTr = false; }
  else memset(&p.tr, 0, sizeof(p.tr));
  ana.suggest(p.tip, TIP_LEN, s, p.state, lastSrc, recent);

  Serial.printf("V:%.2f %.2f %.2f %.2f imb=%.0fmV soc=%.0f%% %s relay=%s risk=%.0f(%s) net=%s q=%u\n",
                s.volt[0], s.volt[1], s.volt[2], s.volt[3], s.imbalance * 1000, s.soc,
                STATE_NAME[p.state], RELAY_NAME[relay.state()], p.risk, Analytics::level(p.risk),
                tel.modeText(), tel.queueDepth());
  tel.consider(p, now);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(BTN_FREEZE, INPUT_PULLUP); pinMode(BTN_RELAYFAULT, INPUT_PULLUP);
  pinMode(BTN_SPIKE, INPUT_PULLUP);  pinMode(BTN_RESET, INPUT_PULLUP);
  pinMode(BTN_OUTAGE, INPUT_PULLUP);
  relay.begin(RELAY_PIN);
  lcd.begin();
  fsm.onTransition = onTr;
  tel.begin();
  Serial.println("BMS started. Set the 4 knobs to ~75% before running.");
}

void loop() {
  uint32_t now = millis();                      // no delay() anywhere: all timing is millis()-based
  handleButtons(now);
  tel.simulateOutage = outageOn;
  tel.update(now);

  bool sampled = false;
  if (now - lastSample >= SAMPLE_MS) { lastSample = now; sampled = true; sampleAndAnalyze(now); }

  // Fast checks every loop
  bool cmd = relay.closed();
  if (mismatch.update(cmd, cmd ^ injRelay, now)) fsm.reportFault(SRC_RELAY, 8, SEV_MAJOR);
  else fsm.clearFault(SRC_RELAY);
#if ENABLE_COMM_FAULT
  if (tel.commLostMs(now) > COMM_FAULT_MS) fsm.reportFault(SRC_COMM, 9, SEV_MINOR);
  else fsm.clearFault(SRC_COMM);
#endif

  fsm.update(now);
  relay.update(now, fsm.state());

  const BmsSnapshot& s = bms.snapshot();
  LcdData d;
  d.soc = s.soc; d.imbMv = s.imbalance * 1000; d.weak = s.weakest; d.strong = s.strongest;
  d.st = fsm.state(); d.relay = relay.closed(); d.faults = (uint16_t)fsm.faultCount();
  d.online = tel.online(); d.rssi = tel.rssi(); d.qDepth = tel.queueDepth(); d.mode = tel.modeText();
  lcd.update(now, d);

  if (sampled || havePendingTr) publish(now);
}
