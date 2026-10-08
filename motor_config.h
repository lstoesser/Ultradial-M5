#pragma once

// Motor configuration page. Select one profile below or add your own entry.
namespace ultradial::motor {
// Angular displacement from activation home at which the ramp spring reaches
// applicationTorqueLimitNm. Rebuild/upload after changing (e.g. 180, 360, 720).
constexpr float kSpringFullTorqueDegrees = 360.0f;
static_assert(kSpringFullTorqueDegrees > 0.0f, "Spring travel must be positive");
enum class ProfileId { Fit1036, GL40KV70, Custom };

struct MotorConfig {
  const char* name;
  int polePairs;
  float nominalBusVoltage;
  float applicationTorqueLimitNm;
  // Used by the ODrive, not this ESP32 firmware. A negative value means it must
  // be supplied from a datasheet or measurement before commissioning.
  float torqueConstantNmPerAmp;
};

constexpr MotorConfig kFit1036 = {
    "DFRobot FIT1036 (24-slot / 22-pole)", 11, 24.0f, 0.030f, -1.0f,
};
constexpr MotorConfig kGL40KV70 = {
    // name, pole pairs, bus voltage, application torque cap (Nm), torque constant
    "GL40 KV70", 14, 16.0f, 0.20f, 0.13023622047244093f,
};
constexpr MotorConfig kCustomMotor = {
    "Custom motor - fill in every field", 7, 12.0f, 0.020f, -1.0f,
};

// ---------- Change this one line to select a motor ----------
constexpr ProfileId kSelectedProfile = ProfileId::GL40KV70;

// Keep this as an expression: PlatformIO's Arduino toolchain defaults to C++11,
// where constexpr functions cannot contain a switch statement.
constexpr MotorConfig kActive =
    kSelectedProfile == ProfileId::Fit1036 ? kFit1036 :
    kSelectedProfile == ProfileId::GL40KV70 ? kGL40KV70 :
    kCustomMotor;
}  // namespace ultradial::motor
