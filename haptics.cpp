// // // #include "Arduino.h"
// // // #include "encoder.h"
// // #include "haptics.h"
// // // #include "gumband_prop.h"
// // #include "motor.h"
// // // #include "lightring.h"
// #include <Arduino.h>

// // static uint32_t last_pub_degrees = 0;
// // static uint8_t haptic_enable = 1;
// // static uint8_t haptic_divisions = 3;        //not being used rn
// // static uint16_t haptic_spacing = 90;       //Distance between haptic ticks in encoder counts (0-4096)
// // static uint8_t haptic_strength = 60; 
// // static uint32_t haptic_duration = 6;
// // static uint32_t haptic_jump = 6;         //Distance from setpoint for trigger
// // static uint32_t counter_previous = 0;

// // uint8_t remaind = 0;                       //remainder
// // uint8_t remainder_rising = 0;             //number of degrees to the next detent clockwise
// // uint8_t remainder_falling = 0;             //number of degrees to the next detent counter clockwise
// // uint8_t prev_rem_rising = 0;
// // uint8_t active_detent = 0;                 //flag so the active detent does not trigger again
// // uint8_t prev_detent = 0;                   //comparison for the active flag 
// // uint8_t scale_factor = 0;
// constexpr float kTwoPi = 2.0f * PI;

// enum class HapticMode { Detents, SpringHome };

// struct HapticProfile {
//   HapticMode mode = HapticMode::Detents;
//   float detentSpacingRad = kTwoPi / 24.0f;
//   float detentStiffnessNmPerRad = 0.090f;
//   float viscousDampingNmPerRadPerSec = 0.0018f;
//   float snapWindowRad = 0.030f;
//   float homeAngleRad = 0.0f;
//   float homeStiffnessNmPerRad = 0.045f;
//   float baselineResistanceNm = 0.0f;
//   bool effectsEnabled = true;
//   float endStopRadians = 0.0f;  // 0 = endless dial
//   float endStopCenterRad = 0.0f;
//   bool detentsEnabled = true;
//   float endStopStiffnessNmPerRad = 0.050f;
// };

// struct Feedback {
//   float positionTurns = 0.0f;
//   float velocityTurnsPerSecond = 0.0f;
//   uint32_t receivedAtUs = 0;
//   uint32_t heartbeatAtUs = 0;
//   uint8_t axisState = 0;
//   bool valid = false;
// };

// HapticProfile profile;
// Feedback feedback;
// uint32_t lastTorqueCommandUs = 0;
// uint32_t lastDiagnosticUs = 0;
// bool torqueModeConfigured = false;


// float nearestDetent(float angle, float spacing) {
//   return roundf(angle / spacing) * spacing;
// }

// float calculateTorqueNm(float angleRad, float velocityRadPerSecond) {
//   float error;
//   float stiffness;
//   if (profile.mode == HapticMode::SpringHome) {
//     // Encoder position is multi-turn: never wrap a wound spring at 360 degrees.
//     error = profile.homeAngleRad - angleRad;
//     stiffness = profile.homeStiffnessNmPerRad;
//   } else if (profile.detentsEnabled) {
//     const float target = nearestDetent(angleRad, profile.detentSpacingRad);
//     error = target - angleRad;
//     stiffness = fabsf(error) < profile.snapWindowRad
//                     ? profile.detentStiffnessNmPerRad * 0.35f
//                     : profile.detentStiffnessNmPerRad;
//   } else {
//     error = 0.0f;
//     stiffness = 0.0f;
//   }

//   // Endless detents only pull toward the nearest click, with no continuous drag.
//   float torque = stiffness * error;
//   if (profile.mode == HapticMode::SpringHome && profile.homeStiffnessNmPerRad > 0.0f) {
//     torque -= profile.viscousDampingNmPerRadPerSec * velocityRadPerSecond;
//   }
//   const float stopOffset = angleRad - profile.endStopCenterRad;
//   if (profile.endStopRadians > 0.0f && fabsf(stopOffset) > profile.endStopRadians) {
//     const float boundary = copysignf(profile.endStopRadians, stopOffset);
//     torque += profile.endStopStiffnessNmPerRad * (boundary - stopOffset);
//   }
//   if (!profile.effectsEnabled) torque = 0.0f;
//   // Smooth friction: opposes motion, settles to zero at rest, and has no home.
//   // The transition near zero avoids abruptly reversing torque on encoder noise.
//   torque -= profile.baselineResistanceNm * tanhf(velocityRadPerSecond / 0.3f);
//   return constrain(torque, -ultradial::motor::kActive.applicationTorqueLimitNm,
//                    ultradial::motor::kActive.applicationTorqueLimitNm);
// }

// bool sendTorque(float torqueNm) {
//   uint8_t payload[8] = {};
//   putFloat(payload, torqueNm);
//   return sendCan(ultradial::odrive::kCmdSetInputTorque, payload, sizeof(payload));
// }

// bool configureTorqueMode() {
//   // Clear any old torque before switching out of position/velocity control.
//   // CAN queues these frames in order; retry if either could not be queued.
//   if (!sendTorque(0.0f)) return false;
//   uint8_t payload[8] = {};
//   const uint32_t controlMode = ultradial::odrive::kControlModeTorque;
//   const uint32_t inputMode = ultradial::odrive::kInputModePassthrough;
//   memcpy(payload, &controlMode, sizeof(controlMode));
//   memcpy(payload + 4, &inputMode, sizeof(inputMode));
//   return sendCan(ultradial::odrive::kCmdSetControllerMode, payload, sizeof(payload));
// }

// void updateHaptics() {
//   const uint32_t nowUs = micros();
//   const bool odriveOnline = feedback.valid &&
//       nowUs - feedback.receivedAtUs <= ultradial::odrive::kFeedbackTimeoutUs;
//   const bool odriveArmed = feedback.heartbeatAtUs != 0 &&
//       nowUs - feedback.heartbeatAtUs <= ultradial::odrive::kHeartbeatTimeoutUs &&
//       feedback.axisState == ultradial::odrive::kAxisStateClosedLoopControl;
//   ultradial::ui::setOdriveStatus(odriveOnline, odriveArmed);
//   if (!odriveOnline || !odriveArmed) torqueModeConfigured = false;
//   ultradial::ui::update();
//   profile.mode = ultradial::ui::springEnabled() ? HapticMode::SpringHome
//                                                 : HapticMode::Detents;
//   profile.effectsEnabled = ultradial::ui::hapticsEnabled();
//   profile.baselineResistanceNm = 0.5f * ultradial::motor::kActive.applicationTorqueLimitNm *
//                                 ultradial::ui::baselineResistance();
//   profile.homeStiffnessNmPerRad = 0.090f * ultradial::ui::springStrength();
//   const int quantity = ultradial::ui::detentQuantity();
//   profile.detentsEnabled = quantity > 0;
//   profile.detentSpacingRad = kTwoPi / (quantity > 0 ? quantity : 1);
//   profile.detentStiffnessNmPerRad = 0.180f * ultradial::ui::detentStrength();
//   profile.snapWindowRad = 0.5f * profile.detentSpacingRad * ultradial::ui::detentWidth();
//   if (!ultradial::ui::hardStopsEnabled()) {
//     profile.endStopRadians = 0.0f;
//   } else if (profile.endStopRadians == 0.0f && odriveOnline) {
//     profile.endStopCenterRad = feedback.positionTurns * kTwoPi;
//     profile.endStopRadians = PI;
//   }

//   if (nowUs - lastDiagnosticUs >= 1000000) {
//     lastDiagnosticUs = nowUs;
//     Serial.printf("CAN=%s axis=%u (%s) haptics=%s mode=%s pos=%.3f turns\n",
//                   odriveOnline ? "online" : "waiting",
//                   feedback.axisState, odriveArmed ? "armed" : "idle",
//                   ultradial::ui::hapticsEnabled() ? "on" : "off",
//                   profile.mode == HapticMode::SpringHome ? "spring" : "detents",
//                   feedback.positionTurns);
//   }

//   if (nowUs - lastTorqueCommandUs < ultradial::odrive::kTorqueCommandIntervalUs) {
//     return;
//   }
//   lastTorqueCommandUs = nowUs;

//   if (odriveOnline && odriveArmed && !torqueModeConfigured) {
//     torqueModeConfigured = configureTorqueMode();
//     return;  // Start haptic torque on the next control tick.
//   }

//   if (!odriveOnline || !odriveArmed) {
//     sendTorque(0.0f);  // Never hold a stale haptic force.
//     return;
//   }
//   sendTorque(calculateTorqueNm(feedback.positionTurns * kTwoPi,
//                                feedback.velocityTurnsPerSecond * kTwoPi));
// }
// // void DialInit()
// // {
// //   EncoderInit();
// //   last_pub_degrees = EncoderGetDegrees();
// //   PublishDialPosition(last_pub_degrees);
// //   counter_previous = EncoderReadPosition();
// // }


// // void UpdateDial(void) 
// // {
// //   //Finish a current motor operation if any is running
// //   UpdateMotor();

// //   uint32_t counter = EncoderReadPosition();
// //   counter = EncoderPositionToDegrees(counter);
// //   scale_factor = floor(255 / (haptic_spacing/2));
// //   remainder_rising = counter % haptic_spacing;
// //   remainder_falling = haptic_spacing - remainder_rising;
// //   if(remainder_rising != prev_rem_rising){
// //     if(remainder_rising > remainder_falling){                  //when you're past halfway to the next detent
// //       PublishHapticRemainderRising(remainder_rising);
// //       PublishHapticRemainderFalling(remainder_falling); 
// //       setBrightness(255-(remainder_falling*scale_factor)); 
// //     }
// //     else if(remainder_rising < remainder_falling){                                                       //flip falling and rising for the first half of the detent spacing to make sure rising only rises and falling only falls
// //       PublishHapticRemainderRising(remainder_rising);
// //       PublishHapticRemainderFalling(remainder_falling);
// //       setBrightness(255-(remainder_rising*scale_factor)); 
// //     }
    
// //     prev_rem_rising = remainder_rising;
// //   }

// //   if(haptic_enable)
// //   {
// //     if (counter > 2){


// //       if(counter > counter_previous)
// //       {
// //         if ((counter % haptic_spacing) < haptic_jump)
// //         {
// //           active_detent = counter / haptic_spacing;
// //           if(active_detent != prev_detent){
// //             BackwardClick(haptic_strength, haptic_duration);
// //             PublishHapticTrigger(BACKWARD_TRIG_VAL);
// //           }
// //         }
// //       }
      
// //       if(counter < counter_previous)
// //       {
// //         if ((counter % haptic_spacing) < haptic_jump)
// //         {
// //           active_detent = counter / haptic_spacing;
// //           if(active_detent != prev_detent){
// //             ForwardClick(haptic_strength, haptic_duration);
// //             PublishHapticTrigger(FORWARD_TRIG_VAL);
// //           }
// //         }
// //       }
// //     }
// //   }

// //   counter_previous = counter;
// //   prev_detent = active_detent;

// //   //Check the current position and update it if we have moved a degree or more
// //   uint32_t current_degrees = counter;//EncoderPositionToDegrees(counter);
// //   if(current_degrees != last_pub_degrees)
// //   {
// //     last_pub_degrees = current_degrees;
// //     PublishDialPosition(last_pub_degrees);
// //   }
// // }


// // //************************
// // // Configuration functions
// // //************************
// // uint8_t GetHapticEnable(void)
// // {
// //   return(haptic_enable);
// // }

// // void EnableHaptic(void)
// // {
// //   haptic_enable = 1;
// // }

// // void DisableHaptic(void)
// // {
// //   haptic_enable = 0;
// // }

// // void SetHapticDivisions(uint8_t divs)
// // {
// //   //NOTE: This does not verify that the selected number of divisions will fit with the currently selected spacing
// //   if (divs > MAX_DIVS_PER_REVOLUTION) 
// //   {
// //     divs = MAX_DIVS_PER_REVOLUTION;
// //   }
// //   haptic_divisions = divs;
// // }

// // void SetHapticSpacing(uint32_t spacing)
// // {
// //   //NOTE: This does not verify that the selected number of divisions will fit
// //   if (spacing > 360) 
// //   {
// //     spacing = 360;
// //   }
// //   haptic_spacing = spacing;
// // }

// // void SetHapticStrength(uint8_t strength)
// // {
// //   haptic_strength = strength;
// // }

// // void SetHapticDuration(uint32_t duration)
// // {
// //   if (duration > MAX_DURATION) 
// //   {
// //     duration = MAX_DIVS_PER_REVOLUTION;
// //   }
// //   haptic_duration = duration;
// // }

// // //The distance from the setpoint before the haptic triggers
// // void SetHapticJumpDistance(uint32_t jump)
// // {
// //   /*
// //   //TODO - Error check this value so it doesn't trigger all the time...
// //   if (jump > MAX_COUNT) 
// //   {
// //     jump = MAX_COUNT;
// //   }
// //   */
// //   haptic_jump = jump;
// // }

// // uint8_t GetHapticDivisions(void)
// // {
// //   return(haptic_divisions);
// // }

// // uint32_t GetHapticSpacing(void)
// // {
// //   return(haptic_spacing);
// // }

// // uint8_t GetHapticStrength(void)
// // {
// //   return(haptic_strength);
// // }

// // uint32_t GetHapticDuration(void)
// // {
// //   return(haptic_duration);
// // }

// // uint32_t GetHapticJumpDistance(void)
// // {
// //   return(haptic_jump);
// // }
