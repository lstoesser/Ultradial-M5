#pragma once

// ODrive / CAN configuration page. Target: M5Stack CoreS3-Lite with Module13.2
// PwrCAN. The module provides the isolated CAN transceiver, so do not add one.
namespace ultradial::odrive {
constexpr uint8_t kAxisNodeId = 0;
constexpr int kCanTxPin = 17;  // CoreS3 M5-Bus pin 16 / PwrCAN CAN_TX
constexpr int kCanRxPin = 18;  // CoreS3 M5-Bus pin 15 / PwrCAN CAN_RX
constexpr uint32_t kCanBitrate = 500000;  // Must match ODrive CAN configuration.
constexpr uint32_t kTorqueCommandIntervalUs = 2000;  // 500 Hz
constexpr uint32_t kFeedbackTimeoutUs = 20000;       // Fail passive after 20 ms
constexpr uint32_t kHeartbeatTimeoutUs = 250000;     // Default heartbeat is 100 ms
constexpr uint8_t kCmdHeartbeat = 0x01;
constexpr uint8_t kCmdGetEncoderEstimates = 0x09;
constexpr uint8_t kCmdSetControllerMode = 0x0B;
constexpr uint32_t kControlModeTorque = 1;
constexpr uint32_t kInputModePassthrough = 1;
constexpr uint8_t kCmdSetInputTorque = 0x0E;
constexpr uint8_t kAxisStateClosedLoopControl = 8;
constexpr uint32_t messageId(uint8_t commandId) {
  return (static_cast<uint32_t>(kAxisNodeId) << 5U) | commandId;
}
}  // namespace ultradial::odrive
