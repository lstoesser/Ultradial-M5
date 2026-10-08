#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <driver/twai.h>

namespace custom_motor {
struct Frame {
  uint32_t id = 0; // Unmodified 11-bit or 29-bit identifier.
  uint8_t length = 0;
  uint8_t data[8] = {};
  bool extended = false;
  bool remote = false;
};
inline bool valid(const Frame& f) {
  return f.length <= 8 && f.id <= (f.extended ? 0x1fffffffUL : 0x7ffUL);
}

struct MotorFeedback {
  uint8_t id = 0; // Lower 8 bits of the received CAN identifier, per supplied format.
  uint8_t status = 0; // 0 = disabled, 1 = enabled, 8..14 = faults.
  float positionRad = 0.0f;
  float velocity = 0.0f; // Supplied specification says "r/s"; confirm units before control.
  float torqueNm = 0.0f;
  int8_t driverTemperatureC = 0;
  int8_t motorTemperatureC = 0;
};
inline const char* motorStatusName(uint8_t status) {
  switch (status) {
    case 0: return "Disabled";
    case 1: return "Enabled";
    case 8: return "Over-voltage";
    case 9: return "Under-voltage";
    case 10: return "Over-current";
    case 11: return "MOS over-temperature";
    case 12: return "Motor over-temperature";
    case 13: return "Communication loss";
    case 14: return "Overload";
    default: return "Unknown status";
  }
}
inline bool decodeMotorFeedback(const Frame& f, MotorFeedback& out) {
  if (!valid(f) || f.remote || f.length != 8) return false;
  const uint16_t position = (uint16_t(f.data[1]) << 8) | f.data[2];
  const uint16_t velocity = (uint16_t(f.data[3]) << 4) | (f.data[4] >> 4);
  const uint16_t torque = (uint16_t(f.data[4] & 0x0f) << 8) | f.data[5];
  out.id = uint8_t(f.id & 0xff);
  out.status = f.data[0] >> 4;
  // Quantized fields are unsigned codes mapped linearly to signed physical units.
  out.positionRad = -12.5f + position * (25.0f / 65535.0f);
  out.velocity = -200.0f + velocity * (400.0f / 4095.0f);
  out.torqueNm = -10.0f + torque * (20.0f / 4095.0f);
  out.driverTemperatureC = f.data[6] < 128 ? f.data[6] : int(f.data[6]) - 256;
  out.motorTemperatureC = f.data[7] < 128 ? f.data[7] : int(f.data[7]) - 256;
  return true;
}
// ESP32 native TWAI transport.
struct ESP32TWAIIntf {
  void (*receive)(const Frame&, void*) = nullptr;
  void* context = nullptr;
};
inline bool sendMsg(ESP32TWAIIntf&, const Frame& f) {
  if (!valid(f)) return false;
  twai_message_t msg = {};
  msg.identifier = f.id;
  msg.extd = f.extended;
  msg.rtr = f.remote;
  msg.data_length_code = f.length;
  if (!f.remote) memcpy(msg.data, f.data, f.length);
  // Success means queued, not acknowledged or executed by the motor.
  return twai_transmit(&msg, 0) == ESP_OK;
}
inline void pumpEvents(ESP32TWAIIntf& intf, int maxEvents = 32) {
  twai_message_t msg;
  // Check the limit BEFORE dequeueing; never consume an extra unprocessed frame.
  while (maxEvents-- > 0 && twai_receive(&msg, 0) == ESP_OK) {
    if (msg.data_length_code > 8) continue;
    Frame f;
    f.id = msg.identifier; f.extended = msg.extd; f.remote = msg.rtr;
    f.length = msg.data_length_code;
    if (!f.remote) memcpy(f.data, msg.data, f.length);
    if (intf.receive) intf.receive(f, intf.context);
  }
}

struct CanIntfWrapper {
  void* context;
  bool (*send)(void*, const Frame&);
  void (*pump)(void*);
  bool sendMsg(const Frame& f) const { return send(context, f); }
  void pumpEvents() const { pump(context); }
};
// Like ODrive's CREATE_CAN_INTF_WRAPPER, but scoped to avoid macro collisions.
template<class TIntf>
CanIntfWrapper wrap_can_intf(TIntf& intf) {
  return {&intf,
    [](void* p, const Frame& f) { return sendMsg(*static_cast<TIntf*>(p), f); },
    [](void* p) { pumpEvents(*static_cast<TIntf*>(p)); }};
}

// Extend this class with GLII commands only after confirming their wire format.
// Call all methods from the same task (not from an ISR).
class MotorCAN {
public:
  enum class RequestState { Idle, Pending, Complete, Timeout, SendFailed };
  using Callback = void (*)(const Frame&, void*);
  using FeedbackCallback = void (*)(const MotorFeedback&, void*);
  explicit MotorCAN(const CanIntfWrapper& intf, uint32_t feedbackId = 0xffffffffUL,
                    bool feedbackExtended = false)
      : intf_(intf), feedbackId_(feedbackId), feedbackExtended_(feedbackExtended) {}
  void onFeedback(FeedbackCallback callback, void* userData = nullptr) {
    feedbackCallback_ = callback;
    feedbackContext_ = userData;
  }
  bool sendFrame(const Frame& f) { return valid(f) && intf_.sendMsg(f); }
  void onFrame(Callback callback, void* context = nullptr) {
    callback_ = callback; context_ = context;
  }
  // Full response ID and length, no assumed node-ID bit shift or RTR convention.
  // Only one outstanding request. ID matching cannot distinguish unsolicited
  // telemetry or a late reply; add payload/sequence matching if GLII supports it.
  bool beginRequest(const Frame& request, uint32_t replyId, bool extended,
                    uint8_t replyLength, uint32_t timeoutMs = 100) {
    if (state_ == RequestState::Pending || replyLength > 8 || timeoutMs == 0 ||
        replyId > (extended ? 0x1fffffffUL : 0x7ffUL)) return false;
    replyId_ = replyId; replyExtended_ = extended; replyLength_ = replyLength;
    startedMs_ = millis(); timeoutMs_ = timeoutMs;
    state_ = RequestState::Pending;
    if (!sendFrame(request)) { state_ = RequestState::SendFailed; return false; }
    return true;
  }
  void update() { expire(); intf_.pumpEvents(); expire(); }
  // Blocking convenience for setup(), similar to ODriveCAN::request().
  // Avoid in a time-critical haptic loop. Response bytes remain protocol-specific.
  bool request(const Frame& command, uint32_t replyId, bool extended,
               uint8_t replyLength, Frame& response, uint32_t timeoutMs = 1000) {
    if (!beginRequest(command, replyId, extended, replyLength, timeoutMs)) return false;
    while (state_ == RequestState::Pending) {
      update();
      if (state_ == RequestState::Pending) delay(1);
    }
    return takeReply(response);
  }
  void onReceive(const Frame& f) {
    if (!valid(f)) return;
    expire();
    if (state_ == RequestState::Pending && !f.remote && f.id == replyId_ &&
        f.extended == replyExtended_ && f.length == replyLength_) {
      reply_ = f; state_ = RequestState::Complete;
    }
    if (feedbackCallback_ && f.id == feedbackId_ && f.extended == feedbackExtended_) {
      MotorFeedback feedback;
      if (decodeMotorFeedback(f, feedback)) feedbackCallback_(feedback, feedbackContext_);
    }
    if (callback_) callback_(f, context_);
  }
  RequestState requestState() const { return state_; }
  bool takeReply(Frame& f) {
    if (state_ != RequestState::Complete) return false;
    f = reply_; state_ = RequestState::Idle; return true;
  }
  void cancelRequest() { state_ = RequestState::Idle; }
private:
  void expire() {
    if (state_ == RequestState::Pending && uint32_t(millis() - startedMs_) >= timeoutMs_)
      state_ = RequestState::Timeout;
  }
  CanIntfWrapper intf_;
  uint32_t feedbackId_;
  bool feedbackExtended_;
  FeedbackCallback feedbackCallback_ = nullptr;
  void* feedbackContext_ = nullptr;
  Callback callback_ = nullptr;
  void* context_ = nullptr;
  RequestState state_ = RequestState::Idle;
  Frame reply_;
  uint32_t replyId_ = 0, startedMs_ = 0, timeoutMs_ = 0;
  uint8_t replyLength_ = 0;
  bool replyExtended_ = false;
};
} // namespace custom_motor
