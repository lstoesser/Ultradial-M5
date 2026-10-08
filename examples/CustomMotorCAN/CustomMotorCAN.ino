#include "MotorCAN.h"
using namespace custom_motor;
ESP32TWAIIntf canIntf;
bool canReady = false;

// Set these to a verified status/heartbeat frame sent by YOUR control board.
// The invalid default deliberately cannot match another device on the bus.
constexpr uint32_t kBoardResponseId = 0xffffffffUL; // TODO: actual received CAN ID
constexpr bool kBoardResponseExtended = false;
constexpr uint8_t kBoardResponseLength = 8;        // Supplied motor reply format
MotorCAN motor(wrap_can_intf(canIntf), kBoardResponseId, kBoardResponseExtended);
struct MotorUserData {
  MotorFeedback lastFeedback;
  bool receivedFeedback = false;
  uint32_t lastFeedbackMs = 0;
  uint32_t lastPrintMs = 0;
  bool printed = false;
};
MotorUserData motorUserData;

void onFeedback(const MotorFeedback& feedback, void* userData) {
  auto* data = static_cast<MotorUserData*>(userData);
  if (!data) return;
  data->lastFeedback = feedback;
  data->receivedFeedback = true;
  data->lastFeedbackMs = millis();
  // A reply confirms presence, not enabled/fault-free state.
  if (!data->printed || uint32_t(data->lastFeedbackMs - data->lastPrintMs) >= 100) {
    data->printed = true;
    data->lastPrintMs = data->lastFeedbackMs;
    Serial.printf("Motor %u: %s (0x%X), pos=%.4f rad, vel=%.3f r/s, torque=%.3f Nm, driver=%d C, motor=%d C\n",
                  feedback.id, motorStatusName(feedback.status), feedback.status,
                  feedback.positionRad, feedback.velocity, feedback.torqueNm,
                  int(feedback.driverTemperatureC), int(feedback.motorTemperatureC));
  }
}

void onCanFrame(const Frame& f, void* context) {
  static_cast<MotorCAN*>(context)->onReceive(f);
}

void logOtherFrame(const Frame& f, void*) {
  if (!f.remote && f.id == kBoardResponseId && f.extended == kBoardResponseExtended && f.length == 8) return;
  Serial.printf("RX %s %s id=0x%lX len=%u", f.extended ? "EXT" : "STD",
                f.remote ? "RTR" : "DATA", (unsigned long)f.id, f.length);
  if (!f.remote) for (uint8_t i = 0; i < f.length; ++i) Serial.printf(" %02X", f.data[i]);
  Serial.println();
}

void waitForControlBoard() {
  motorUserData.receivedFeedback = false;
  Serial.println("Waiting for control board...");
  if (kBoardResponseId > (kBoardResponseExtended ? 0x1fffffffUL : 0x7ffUL) ||
      kBoardResponseLength > 8) {
    Serial.println("Configure kBoardResponseId / Extended / Length first. RX frames are logged below.");
  }
  uint32_t lastMessageMs = millis();
  while (!motorUserData.receivedFeedback) {
    motor.update(); // Drains TWAI RX queue and invokes our frame callback.
    if (uint32_t(millis() - lastMessageMs) >= 5000) {
      lastMessageMs = millis();
      Serial.println("Still waiting for the configured control-board response...");
    }
    delay(1); // Yield to ESP32 background tasks while waiting indefinitely.
  }
  Serial.println("Found control board");
}

void setup() {
  Serial.begin(115200);
  canIntf.receive = onCanFrame;
  canIntf.context = &motor;
  // Register decoded feedback once; motor.update() dispatches future replies.
  motor.onFeedback(onFeedback, &motorUserData);
  motor.onFrame(logOtherFrame); // Optional raw logging for unrecognized frames.
  // Match your already-working pins/bitrate. These are the existing CoreS3 defaults.
  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_18, TWAI_MODE_NORMAL);
  general.rx_queue_len = 32;
  general.alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_RX_QUEUE_FULL |
                           TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED;
  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t result = twai_driver_install(&general, &timing, &filter);
  if (result == ESP_OK) result = twai_start();
  canReady = result == ESP_OK;
  Serial.printf("CAN start: %d\n", result);
  if (!canReady) {
    Serial.println("CAN initialization failed; setup cannot proceed.");
    while (true) delay(100);
  }
  waitForControlBoard();
  // Put initialization that requires the board to be present below this line.
  // This waits for periodic status. If your board only responds to queries,
  // send its VERIFIED status query first; no ODrive command IDs are assumed.
  // This example sends no motor commands. Implement verified GLII commands with
  // motor.sendFrame(frame), or motor.beginRequest(...) and motor.takeReply(...).
}
void loop() {
  if (!canReady) { delay(100); return; }
  motor.update();
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    const uint32_t faults = alerts & (TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_FAILED);
    if (faults) Serial.printf("TWAI alerts: 0x%lX\n", (unsigned long)faults);
  }
}

