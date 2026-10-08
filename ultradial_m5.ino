#include <Arduino.h>
#include <M5Unified.h>
#include <driver/twai.h>

#include "encoder_config.h"
#include "motor_config.h"
#include "odrive_config.h"
#include "dial_ui.h"
#include "inertia.h"
#include "baseline_resistance.h"
#include "ODriveCAN.h"
#include "ODriveESP32TWAI.hpp"
// #include "haptics.h"

// The ODrive Micro closes the FOC current/commutation loop. This ESP32 haptic host
// receives position feedback over CAN and sends torque requests. Motor/encoder
// changes therefore live entirely in include/config/, not in the haptic algorithm.

constexpr float kTwoPi = 2.0f * PI;

ESP32TWAIIntf can_intf;

enum class HapticMode { Detents, SpringHome };

struct HapticProfile {
  HapticMode mode = HapticMode::Detents;
  float detentSpacingRad = kTwoPi / 24.0f;
  float detentStiffnessNmPerRad = 0.090f;
  float viscousDampingNmPerRadPerSec = 0.0018f;
  float snapWindowRad = 0.030f;
  float homeAngleRad = 0.0f;
  float homeStiffnessNmPerRad = 0.045f;
  bool springTurnStop = false;
  float springTurnCenterRad = 0.0f;
  float baselineResistanceNm = 0.0f;
  float inertiaStrength = 0.0f;
  bool effectsEnabled = true;
  float endStopRadians = 0.0f;  // 0 = endless dial
  float endStopCenterRad = 0.0f;
  bool detentsEnabled = true;
  float endStopStiffnessNmPerRad = 0.050f;
};
HapticProfile profile;
CoastAssist coast;
BaselineResistance baseline;
// Instantiate ODrive objects
ODriveCAN odrv0(wrap_can_intf(can_intf), ultradial::odrive::kAxisNodeId); // Standard CAN message ID
ODriveCAN* odrives[] = {&odrv0}; // Make sure all ODriveCAN instances are accounted for here

struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};


uint32_t lastTorqueCommandUs = 0;
uint32_t lastDiagnosticUs = 0;
bool torqueModeConfigured = false;
uint32_t lastFeedbackUs = 0;
uint32_t lastHeartbeatUs = 0;
// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
// Called every time a Heartbeat message arrives from the ODrive
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
  lastHeartbeatUs = micros();
}

// Called every time a feedback message arrives from the ODrive
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  if (!isfinite(msg.Pos_Estimate) || !isfinite(msg.Vel_Estimate)) return;
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
  lastFeedbackUs = micros();
}

// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive: odrives) {
    onReceive(msg, *odrive);
  }
}

twai_timing_config_t timingForConfiguredBitrate() {
  switch (ultradial::odrive::kCanBitrate) {
    case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
    case 500000: return TWAI_TIMING_CONFIG_500KBITS();
    case 250000: return TWAI_TIMING_CONFIG_250KBITS();
    case 125000: return TWAI_TIMING_CONFIG_125KBITS();
    default:
      Serial.println("Unsupported CAN bitrate; falling back to 500 kbit/s.");
      return TWAI_TIMING_CONFIG_500KBITS();
  }
}

void configureCan() {
  const twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(ultradial::odrive::kCanTxPin),
      static_cast<gpio_num_t>(ultradial::odrive::kCanRxPin), TWAI_MODE_NORMAL);
  const twai_timing_config_t timing = timingForConfiguredBitrate();
  const twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  const esp_err_t installResult = twai_driver_install(&general, &timing, &filter);
  if (installResult != ESP_OK || twai_start() != ESP_OK) {
    Serial.printf("CAN start failed: %d. Motor output remains disabled.\n", installResult);
    while (true) delay(1000);
  }
}

float nearestDetent(float angle, float spacing) {
  return roundf(angle / spacing) * spacing;
}

float calculateTorqueNm(float angleRad, float velocityRadPerSecond, float dt) {
  float error;
  float stiffness;
  if (profile.mode == HapticMode::SpringHome) {
    // Encoder position is multi-turn: never wrap a wound spring at 360 degrees.
    error = (profile.springTurnStop ? profile.springTurnCenterRad : profile.homeAngleRad) - angleRad;
    // Rotary spring: torque = stiffness * displacement; max at configured angle.
    stiffness = profile.springTurnStop
        ? ultradial::motor::kActive.applicationTorqueLimitNm /
          (ultradial::motor::kSpringFullTorqueDegrees * PI / 180.0f)
        : profile.homeStiffnessNmPerRad;
  } else if (profile.detentsEnabled) {
    const float target = nearestDetent(angleRad, profile.detentSpacingRad);
    error = target - angleRad;
    // Sine force is zero at the click center and the midpoint between clicks.
    // Smooth the soft-center gain too, avoiding the old stiffness step.
    const float distance = fabsf(error);
    const float t = profile.snapWindowRad > 0.0f
        ? constrain(distance / profile.snapWindowRad, 0.0f, 1.0f) : 1.0f;
    const float gain = 0.35f + 0.65f * t * t * (3.0f - 2.0f * t);
    stiffness = profile.detentStiffnessNmPerRad * gain;
    error = profile.detentSpacingRad / kTwoPi *
            sinf(kTwoPi * error / profile.detentSpacingRad);
  } else {
    error = 0.0f;
    stiffness = 0.0f;
  }

  // Endless detents only pull toward the nearest click, with no continuous drag.
  float torque = stiffness * error;
  if (profile.mode == HapticMode::SpringHome && profile.homeStiffnessNmPerRad > 0.0f) {
    torque -= profile.viscousDampingNmPerRadPerSec * velocityRadPerSecond;
  }
  const float stopOffset = angleRad - profile.endStopCenterRad;
  if (profile.endStopRadians > 0.0f && fabsf(stopOffset) > profile.endStopRadians) {
    const float boundary = copysignf(profile.endStopRadians, stopOffset);
    torque += profile.endStopStiffnessNmPerRad * (boundary - stopOffset);
  }
  if (!profile.effectsEnabled) torque = 0.0f;
  torque += baseline.update(velocityRadPerSecond, profile.baselineResistanceNm, dt);
  // Do not push through an enabled end stop.
  if (profile.effectsEnabled && profile.endStopRadians > 0.0f &&
      fabsf(stopOffset) >= profile.endStopRadians) {
    coast.reset();
  } else {
    torque += coast.update(velocityRadPerSecond, profile.inertiaStrength, dt,
                           ultradial::motor::kActive.applicationTorqueLimitNm);
  }
  if (profile.effectsEnabled && profile.mode == HapticMode::SpringHome && profile.springTurnStop) {
    const float offset = angleRad - profile.springTurnCenterRad;
    const float fullTorqueRad = ultradial::motor::kSpringFullTorqueDegrees * PI / 180.0f;
    const float fraction = constrain(fabsf(offset) / fullTorqueRad, 0.0f, 1.0f);
    const float springTorque = -copysignf(
        ultradial::motor::kActive.applicationTorqueLimitNm * fraction, offset);
    // Fade auxiliary forces over the entire travel so they cannot cancel the
    // full-force endpoint. The spring itself remains exactly linear in angle.
    const float auxiliaryTorque = torque - stiffness * error;
    torque = springTorque + auxiliaryTorque * (1.0f - fraction);
    if (fraction >= 1.0f) coast.reset();
  }
  return constrain(torque, -ultradial::motor::kActive.applicationTorqueLimitNm,
                   ultradial::motor::kActive.applicationTorqueLimitNm);
}

bool configureTorqueMode() {
  // Clear any old torque before switching out of position/velocity control.
  // CAN queues these frames in order; retry if either could not be queued.
  if (!odrv0.setTorque(0.0f)) return false;
  return odrv0.setControllerMode(ultradial::odrive::kControlModeTorque, ultradial::odrive::kInputModePassthrough);
}

bool feedbackFresh(uint32_t nowUs) {
  return odrv0_user_data.received_feedback &&
      nowUs - lastFeedbackUs <= ultradial::odrive::kFeedbackTimeoutUs;
}

bool odriveOnline(uint32_t nowUs) {
  return odrv0_user_data.received_heartbeat &&
      nowUs - lastHeartbeatUs <= ultradial::odrive::kHeartbeatTimeoutUs;
}

bool odriveArmed(uint32_t nowUs) {
  return odriveOnline(nowUs) &&
      odrv0_user_data.last_heartbeat.Axis_State == ultradial::odrive::kAxisStateClosedLoopControl;
}

// UI and settings only; no CAN torque commands here.
void updateUiAndSettings() {
  const uint32_t nowUs = micros();
  ultradial::ui::setOdriveStatus(odriveOnline(nowUs), odriveArmed(nowUs));
  ultradial::ui::update();
  profile.mode = ultradial::ui::springEnabled() ? HapticMode::SpringHome
                                                : HapticMode::Detents;
  profile.effectsEnabled = ultradial::ui::hapticsEnabled();
  profile.inertiaStrength = ultradial::ui::inertiaEnabled()
      ? ultradial::ui::inertiaStrength() : 0.0f;
  if (profile.inertiaStrength == 0.0f) coast.reset();
  profile.baselineResistanceNm = 0.5f * ultradial::motor::kActive.applicationTorqueLimitNm *
                                ultradial::ui::baselineResistance();
  profile.homeStiffnessNmPerRad = 0.090f * ultradial::ui::springStrength();
  const bool turnStopRequested = ultradial::ui::springTurnStopEnabled() &&
      profile.mode == HapticMode::SpringHome && profile.effectsEnabled;
  if (!turnStopRequested) {
    profile.springTurnStop = false;
  } else if (!profile.springTurnStop && feedbackFresh(nowUs)) {
    profile.springTurnCenterRad = odrv0_user_data.last_feedback.Pos_Estimate * kTwoPi;
    profile.springTurnStop = true;
    coast.reset();
  }
  const int quantity = ultradial::ui::detentQuantity();
  profile.detentsEnabled = quantity > 0;
  profile.detentSpacingRad = kTwoPi / (quantity > 0 ? quantity : 1);
  profile.detentStiffnessNmPerRad = 0.180f * ultradial::ui::detentStrength();
  profile.snapWindowRad = 0.5f * profile.detentSpacingRad * ultradial::ui::detentWidth();


  if (!ultradial::ui::hardStopsEnabled()) {
    profile.endStopRadians = 0.0f;
  } else if (profile.endStopRadians == 0.0f && feedbackFresh(nowUs)) {
    profile.endStopCenterRad = odrv0_user_data.last_feedback.Pos_Estimate * kTwoPi;
    profile.endStopRadians = PI;
  }

}

// Real-time haptic profile evaluation, diagnostics and CAN output.
void updateHapticProfile() {
  const uint32_t nowUs = micros();
  const bool online = odriveOnline(nowUs);
  const bool armed = odriveArmed(nowUs);
  const bool fresh = feedbackFresh(nowUs);
  if (!online || !armed || !fresh) {
    torqueModeConfigured = false;
    coast.reset();
    baseline.reset();
  }
  if (nowUs - lastDiagnosticUs >= 1000000) {
    lastDiagnosticUs = nowUs;
    Serial.printf("CAN=%s feedback=%s axis=%u (%s) haptics=%s mode=%s pos=%.3f turns\n",
                  online ? "online" : "waiting",
                  fresh ? "fresh" : "stale",
                  odrv0_user_data.last_heartbeat.Axis_State, armed ? "armed" : "idle",
                  ultradial::ui::hapticsEnabled() ? "on" : "off",
                  profile.mode == HapticMode::SpringHome ? "spring" : "detents",
                  odrv0_user_data.last_feedback.Pos_Estimate);
  }

  if (nowUs - lastTorqueCommandUs < ultradial::odrive::kTorqueCommandIntervalUs) {
    return;
  }
  const float dt = (nowUs - lastTorqueCommandUs) * 1e-6f;
  lastTorqueCommandUs = nowUs;

  static bool armRequested = false;
  if (!online) armRequested = false;
  if (armed) armRequested = true;  // Do not undo a later deliberate disarm.
  if (online && fresh && !armed && !armRequested &&
      odrv0_user_data.last_heartbeat.Axis_State == ODriveAxisState::AXIS_STATE_IDLE &&
      odrv0_user_data.last_heartbeat.Axis_Error == 0) {
    if (configureTorqueMode()) {
      armRequested = odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
    return;
  }

  if (online && armed && fresh && !torqueModeConfigured) {
    torqueModeConfigured = configureTorqueMode();
    return;  // Start haptic torque on the next control tick.
  }

  if (!online || !armed || !fresh) {
    odrv0.setTorque(0.0f);  // Never hold a stale haptic force.
    return;
  }
  odrv0.setTorque(calculateTorqueNm(odrv0_user_data.last_feedback.Pos_Estimate * kTwoPi,
                               odrv0_user_data.last_feedback.Vel_Estimate * kTwoPi, dt));
}

void handleConsole() {
  while (Serial.available()) {
    switch (static_cast<char>(Serial.read())) {
      case 'd': ultradial::ui::setSpringEnabled(false); break;
      case 'h': ultradial::ui::setSpringEnabled(true); break;
      case '0': ultradial::ui::setDetentQuantity(12); break;
      case '1': ultradial::ui::setDetentQuantity(24); break;
      case '2': ultradial::ui::setDetentQuantity(48); break;
    }
  }
}


void setup() {
  auto cfg = M5.config();
  M5.begin(cfg); // Initialize CoreS3 hardware
  Serial.begin(115200);
  delay(300);
  Serial.println("UltraDial booting");
  ultradial::ui::begin();
  Serial.printf("UltraDial: %s | %s\n", ultradial::motor::kActive.name,
                ultradial::encoder::kActive.name);

  // Register callbacks for the heartbeat and encoder feedback messages
  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  configureCan();

  Serial.println("Waiting for ODrive heartbeat and encoder feedback; UI remains active.");
}


void loop() {
  pumpEvents(can_intf);
  handleConsole();
  updateUiAndSettings();
  // Display writes can take time; refresh CAN before evaluating feedback age.
  pumpEvents(can_intf);
  updateHapticProfile();
}
