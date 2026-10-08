#pragma once
#include <M5Unified.h>
#include <Wire.h>
#include <gl-ii-CAN.h>
#include <esp_system.h>
#include "ControlConfig.h"
#include "Telemetry.h"

namespace controller {
constexpr uint16_t BG = 0x0842, CARD = 0x18C5, WHITE = 0xEF7D, MUTED = 0x94B2;
constexpr uint16_t ACCENT = 0x36DF, RED = 0xF986, GRID = 0x2947;
struct State {
  float command[5] = {}, measured[3] = {};
  bool requested = false, active = false, seen = false, canReady = false, busOff = false;
  uint32_t lastRx = 0, rxCount = 0, errors = 0;
  uint8_t status = 255, reason = telemetry::Starting, pendingSpecial = 0;
};
State state;
SemaphoreHandle_t mutex;
M5Canvas screen(&M5.Display);
// M5Unified reserves I2C1 for internal touch/power. External Port A is I2C0.
TwoWire link(0);
portMUX_TYPE packetLock = portMUX_INITIALIZER_UNLOCKED;
uint8_t cached[telemetry::SIZE] = {};
uint32_t lastRequest = 0;
bool requestSeen = false, ready = false, i2cReady = false;
uint32_t session = 0, sequence = 0, lastPaint = 0;
int dragging = -1, selected = 1;

glii::Config motorConfig() {
  glii::Config c;
  c.txPin = config::CAN_TX; c.rxPin = config::CAN_RX;
  c.bitrate = config::CAN_BITRATE; c.motorId = config::MOTOR_ID; c.feedbackId = config::FEEDBACK_ID;
  c.positionRange = config::POSITION_RANGE; c.velocityRange = config::VELOCITY_RANGE;
  c.torqueRange = config::TORQUE_RANGE; return c;
}
glii::GLIICAN motor(motorConfig());
State snapshot() {
  xSemaphoreTake(mutex, portMAX_DELAY); State s = state; xSemaphoreGive(mutex); return s;
}
void onFeedback(const glii::MotorFeedback& f, void*) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.measured[0] = f.positionRad; state.measured[1] = f.velocity; state.measured[2] = f.torqueNm;
  state.status = f.status; state.seen = true; state.lastRx = millis(); ++state.rxCount;
  xSemaphoreGive(mutex);
}
void publish() {
  State s = snapshot(); uint32_t now = millis();
  telemetry::Sample t;
  t.session = session; t.sequence = ++sequence; t.sourceMs = now;
  uint32_t age = s.seen ? now - s.lastRx : UINT32_MAX;
  t.feedbackAge = age < 65535 ? age : 65535;
  t.flags = (age <= config::FEEDBACK_TIMEOUT_MS ? telemetry::Fresh : 0) |
    (s.active ? telemetry::Active : 0) | (s.requested ? telemetry::Requested : 0) |
    (s.canReady ? telemetry::CanReady : 0);
  t.motorStatus = s.status; t.reason = s.reason; t.rxCount = s.rxCount; t.errors = s.errors;
  memcpy(t.measured, s.measured, sizeof(t.measured)); memcpy(t.command, s.command, sizeof(t.command));
  uint8_t packet[telemetry::SIZE]; telemetry::pack(t, packet);
  portENTER_CRITICAL(&packetLock); memcpy(cached, packet, sizeof(packet)); portEXIT_CRITICAL(&packetLock);
}
// I2C callback serves a prebuilt snapshot only. No CAN, screen work or mutex waits.
void onRequest() {
  uint8_t packet[telemetry::SIZE];
  portENTER_CRITICAL(&packetLock);
  memcpy(packet, cached, sizeof(packet)); lastRequest = millis(); requestSeen = true;
  portEXIT_CRITICAL(&packetLock);
  link.write(packet, sizeof(packet));
}
void stop(uint8_t reason) { // CAN task only
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.requested = state.active = false; state.reason = reason;
  xSemaphoreGive(mutex);
  motor.discardPendingCommands(); motor.special(0xFD);
}
void canWorker(void*) {
  motor.onFeedback(onFeedback);
  bool initialized = motor.begin();
  xSemaphoreTake(mutex, portMAX_DELAY);
  state.canReady = initialized; state.reason = initialized ? telemetry::Stopped : telemetry::InitFailed;
  xSemaphoreGive(mutex);
  if (initialized) motor.special(0xFD);
  bool running = false, busOff = false;
  uint32_t started = 0, lastStop = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    if (initialized) {
      motor.update(64);
      auto events = motor.readBusEvents();
      if (events.busOff) busOff = true;
      if (events.txFailed || events.rxOverflow) {
        xSemaphoreTake(mutex, portMAX_DELAY); ++state.errors; xSemaphoreGive(mutex);
        if (running) { stop(telemetry::CanError); running = false; }
      }
      xSemaphoreTake(mutex, portMAX_DELAY);
      uint8_t op = state.pendingSpecial; state.pendingSpecial = 0;
      bool allowSpecial = !state.requested && !state.active;
      xSemaphoreGive(mutex);
      if (!busOff && op && allowSpecial && !motor.special(op)) stop(telemetry::SpecialFailed);
      State s = snapshot(); uint32_t now = millis();
      if (busOff) {
        if (!s.busOff) stop(telemetry::BusOff);
        running = false;
        xSemaphoreTake(mutex, portMAX_DELAY); state.busOff = true; xSemaphoreGive(mutex);
      } else if (!s.requested) {
        if (running || now - lastStop >= 100) {
          if (running) motor.discardPendingCommands();
          motor.special(0xFD); lastStop = now;
        }
        running = false;
      } else {
        if (!running) {
          if (!motor.special(0xFC)) stop(telemetry::SendFailed);
          else { running = true; started = now; }
        }
        if (running) {
          bool fresh = s.seen && now - s.lastRx <= config::FEEDBACK_TIMEOUT_MS;
          if (fresh && s.status >= 8) { stop(telemetry::MotorFault); running = false; }
          else if (now - started > config::FEEDBACK_TIMEOUT_MS && (!fresh || s.status != 1)) {
            stop(telemetry::FeedbackLost); running = false;
          } else if (!motor.mit(s.command[0], s.command[1], s.command[3], s.command[4], s.command[2])) {
            stop(telemetry::SendFailed); running = false;
          }
        }
      }
      xSemaphoreTake(mutex, portMAX_DELAY);
      state.active = running;
      if (running) state.reason = state.seen && state.status == 1 && now - state.lastRx <= config::FEEDBACK_TIMEOUT_MS
        ? telemetry::Running : telemetry::Waiting;
      xSemaphoreGive(mutex);
    }
    publish(); // Independent of whether Tab5 is attached or responding.
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(config::COMMAND_MS));
  }
}
void requestRun(bool run) {
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (!run) { state.requested = false; state.reason = telemetry::Stopped; }
  else if (state.canReady && !state.busOff) state.requested = true;
  xSemaphoreGive(mutex);
}
void setValue(int i, float value) {
  auto r = config::SLIDERS[i];
  value = fmaxf(r.lo, fminf(r.hi, roundf(value / config::STEPS[i]) * config::STEPS[i]));
  xSemaphoreTake(mutex, portMAX_DELAY); state.command[i] = value; xSemaphoreGive(mutex);
}
void serialCommand(int c) {
  if (c == 'e') requestRun(true);
  if (c == 'x') requestRun(false);
  xSemaphoreTake(mutex, portMAX_DELAY);
  if (state.canReady && !state.busOff && !state.active && !state.requested) {
    if (c == 'z') state.pendingSpecial = 0xFE;
    if (c == 'c') state.pendingSpecial = 0xFB;
  }
  xSemaphoreGive(mutex);
}
void touch() {
  auto t = M5.Touch.getDetail();
  if (!t.isPressed()) { dragging = -1; return; }
  if (t.wasPressed()) {
    if (t.y >= 198 && t.y < 228) {
      if (t.x < 82) requestRun(true);
      else if (t.x < 170) requestRun(false);
      else {
        State s = snapshot();
        setValue(selected, s.command[selected] + (t.x < 245 ? -1 : 1) * config::STEPS[selected]);
      }
    }
    for (int i = 0; i < 5; ++i) {
      int y = 32 + i * 32;
      if (t.y >= y && t.y < y + 32) {
        selected = i;
        if (t.y >= y + 10 && t.x >= 12 && t.x <= 308) dragging = i;
      }
    }
  }
  if (dragging >= 0) {
    auto r = config::SLIDERS[dragging];
    setValue(dragging, r.lo + constrain(int(t.x) - 18, 0, 284) * (r.hi - r.lo) / 284.0f);
  }
}
void label(int x, int y, const char* str, uint16_t color = WHITE, int size = 1) {
  screen.setTextSize(size); screen.setTextColor(color); screen.setCursor(x, y); screen.print(str);
}
void draw() {
  State s = snapshot(); screen.fillSprite(BG);
  label(8, 2, "MIT MOTOR TUNER", WHITE, 2);
  label(8, 21, telemetry::reasonName(s.reason), s.active ? ACCENT : MUTED);
  for (int i = 0; i < 5; ++i) {
    int y = 32 + i * 32;
    screen.fillRoundRect(4, y, 312, 30, 4, CARD);
    label(10, y + 2, config::LABELS[i], selected == i ? ACCENT : WHITE);
    char value[24]; snprintf(value, sizeof(value), "%+.3f", s.command[i]); label(251, y + 2, value);
    screen.drawFastHLine(18, y + 21, 284, GRID);
    int x = 18 + int(284 * (s.command[i] - config::SLIDERS[i].lo) / (config::SLIDERS[i].hi - config::SLIDERS[i].lo));
    screen.drawFastHLine(18, y + 21, max(1, x - 18), ACCENT); screen.fillCircle(x, y + 21, 7, WHITE);
  }
  screen.fillRoundRect(4, 198, 76, 30, 4, s.requested ? GRID : 0x240A);
  screen.fillRoundRect(84, 198, 82, 30, 4, RED);
  screen.fillRoundRect(170, 198, 71, 30, 4, GRID); screen.fillRoundRect(245, 198, 71, 30, 4, GRID);
  label(18, 206, s.requested ? "ON" : "RUN", WHITE, 2); label(100, 206, "STOP", WHITE, 2);
  label(198, 206, "-", WHITE, 2); label(274, 206, "+", WHITE, 2);
  portENTER_CRITICAL(&packetLock); uint32_t request = lastRequest; bool seen = requestSeen; portEXIT_CRITICAL(&packetLock);
  char status[54]; snprintf(status, sizeof(status), "Tab:%s CAN:%u RX:%lu", !i2cReady ? "I2C ERR" :
    (seen && millis() - request < 1000 ? "linked" : "offline"), s.status, (unsigned long)s.rxCount);
  label(8, 232, status, MUTED);
  screen.pushSprite(0, 0);
}
bool validConfig() {
  const float lo[5] = {-config::POSITION_RANGE, -config::VELOCITY_RANGE, -config::TORQUE_RANGE, 0, 0};
  const float hi[5] = {config::POSITION_RANGE, config::VELOCITY_RANGE, config::TORQUE_RANGE, 500, 5};
  for (int i = 0; i < 5; ++i) {
    auto r = config::SLIDERS[i];
    if (!isfinite(r.lo) || !isfinite(r.hi) || !isfinite(config::STEPS[i]) ||
        !(r.lo < r.hi && r.lo >= lo[i] && r.hi <= hi[i] && r.lo <= 0 && r.hi >= 0 && config::STEPS[i] > 0)) return false;
  }
  return true;
}
void begin() {
  Serial.begin(115200);
  auto cfg = M5.config(); cfg.external_imu = false; cfg.external_rtc = false; M5.begin(cfg);
  if (M5.Display.width() < M5.Display.height()) M5.Display.setRotation(M5.Display.getRotation() ^ 1);
  if (config::FLIP_180) M5.Display.setRotation((M5.Display.getRotation() + 2) % 4);
  M5.Display.setTextSize(2);
  if (M5.Display.width() != 320 || M5.Display.height() != 240) { M5.Display.println("CoreS3 required"); return; }
  if (!validConfig()) { M5.Display.println("Invalid slider limits"); return; }
  mutex = xSemaphoreCreateMutex(); screen.setColorDepth(16);
  if (!mutex || !screen.createSprite(320, 240)) { M5.Display.println("Allocation failed"); return; }
  session = esp_random(); publish();
  // Do not steal the internal bus used by the touchscreen / PMIC.
  if (M5.Ex_I2C.getPort() == 0 && M5.In_I2C.getPort() != 0) {
    M5.Ex_I2C.release();
    link.setBufferSize(128); link.onRequest(onRequest);
    i2cReady = link.begin(uint8_t(telemetry::ADDRESS), config::I2C_SDA, config::I2C_SCL, telemetry::CLOCK_HZ);
  }
  if (xTaskCreate(canWorker, "motorCAN", 4096, nullptr, 3, nullptr) != pdPASS) {
    M5.Display.println("CAN task failed"); return;
  }
  ready = true;
}
void update() {
  M5.update(); if (!ready) { delay(10); return; }
  touch(); if (Serial.available()) serialCommand(Serial.read());
  if (millis() - lastPaint >= 50) { lastPaint = millis(); draw(); }
  delay(1);
}
}
