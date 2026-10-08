#include <Arduino.h>
#include "driver/twai.h"

// CoreS3: PwrCAN DIP routing must select M5-Bus TX=16, RX=15.
// For Tab5 with that same routing, use GPIO_NUM_6 / GPIO_NUM_7.
constexpr gpio_num_t TX_PIN = GPIO_NUM_17;
constexpr gpio_num_t RX_PIN = GPIO_NUM_18;
constexpr uint16_t MOTOR_ID = 1;
// Must match the motor's saved bitrate; 500 kbit/s is inherited from your sketch.
constexpr uint32_t BITRATE = 500000;
// Match these to the motor's parameter page. These are manual defaults.
constexpr float P_RANGE = 12.5f, V_RANGE = 200.0f, T_RANGE = 10.0f;
bool enabled = false;
uint32_t lastCommand = 0, lastStatus = 0;

bool sendFrame(const uint8_t *data) {
  twai_message_t tx = {};
  tx.identifier = MOTOR_ID;  // MIT mode: standard ID, mode bits = 0
  tx.data_length_code = 8;
  tx.ss = 1;  // Diagnostic: do not retry a failed command indefinitely.
  memcpy(tx.data, data, 8);
  esp_err_t err = twai_transmit(&tx, pdMS_TO_TICKS(10));
  if (err != ESP_OK) Serial.printf("TX enqueue: %s\n", esp_err_to_name(err));
  return err == ESP_OK; // Enqueued, not proof of CAN acknowledgement.
}

bool special(uint8_t command) {
  uint8_t data[8];
  memset(data, 0xFF, sizeof(data));
  data[7] = command;
  return sendFrame(data);
}

uint16_t encode(float value, float lo, float hi, unsigned bits) {
  value = constrain(value, lo, hi);
  return (uint16_t)((value - lo) * ((1UL << bits) - 1) / (hi - lo));
}

void mit(float position, float velocity, float kp, float kd, float torque) {
  uint16_t p = encode(position, -P_RANGE, P_RANGE, 16);
  uint16_t v = encode(velocity, -V_RANGE, V_RANGE, 12);
  uint16_t k = encode(kp, 0, 500, 12);
  uint16_t d = encode(kd, 0, 5, 12);
  uint16_t t = encode(torque, -T_RANGE, T_RANGE, 12);
  uint8_t data[8] = {uint8_t(p >> 8), uint8_t(p), uint8_t(v >> 4),
    uint8_t((v << 4) | (k >> 8)), uint8_t(k), uint8_t(d >> 4),
    uint8_t((d << 4) | (t >> 8)), uint8_t(t)};
  sendFrame(data);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(TX_PIN, RX_PIN, TWAI_MODE_NORMAL);
  g.alerts_enabled = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED |
    TWAI_ALERT_BUS_ERROR | TWAI_ALERT_BUS_OFF | TWAI_ALERT_RX_QUEUE_FULL;
  twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();
  if (BITRATE == 1000000) t = TWAI_TIMING_CONFIG_1MBITS();
  else if (BITRATE != 500000) {
    Serial.println("Configure a supported bitrate: 500000 or 1000000.");
    while (true) delay(1000);
  }
  twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  esp_err_t err = twai_driver_install(&g, &t, &f);
  if (err == ESP_OK) err = twai_start();
  Serial.printf("TWAI startup: %s\n", esp_err_to_name(err));
  if (err != ESP_OK) while (true) delay(1000);
  Serial.println("e: enable MIT with zero gains/torque; x: exit; c: clear fault.");
  Serial.println("No automatic enable. Power-cycle/reset after correcting bus-off cause.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'e') enabled = special(0xFC);
    if (c == 'x') { enabled = false; special(0xFD); }
    if (c == 'c') special(0xFB);
  }
  if (enabled && millis() - lastCommand >= 10) {
    lastCommand = millis();
    mit(0, 0, 0, 0, 0); // Communication test; no requested holding or drive torque.
  }
  twai_message_t rx = {};
  while (twai_receive(&rx, 0) == ESP_OK) {
    Serial.printf("RX %s ID=%03lX DLC=%u%s:", rx.extd ? "EXT" : "STD",
      (unsigned long)rx.identifier, rx.data_length_code, rx.rtr ? " RTR" : "");
    if (!rx.rtr) for (unsigned i = 0; i < rx.data_length_code && i < 8; ++i)
      Serial.printf(" %02X", rx.data[i]);
    Serial.println();
  }
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) { enabled = false; Serial.println("BUS OFF"); }
    if (alerts & TWAI_ALERT_TX_FAILED) Serial.println("TX failed: check ACK/wiring/bitrate");
    if (alerts & TWAI_ALERT_TX_SUCCESS) Serial.println("TX acknowledged on CAN bus");
    if (alerts & TWAI_ALERT_BUS_ERROR) Serial.println("CAN bus error");
    if (alerts & TWAI_ALERT_RX_QUEUE_FULL) Serial.println("RX queue full");
  }
  if (millis() - lastStatus >= 1000) {
    lastStatus = millis();
    twai_status_info_t s = {};
    if (twai_get_status_info(&s) == ESP_OK)
      Serial.printf("state=%d TEC=%lu REC=%lu failed=%lu busErrors=%lu\n", (int)s.state,
        (unsigned long)s.tx_error_counter, (unsigned long)s.rx_error_counter,
        (unsigned long)s.tx_failed_count, (unsigned long)s.bus_error_count);
  }
  delay(1);
}
