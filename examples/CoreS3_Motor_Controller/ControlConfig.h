#pragma once
#include <driver/gpio.h>
namespace config {
constexpr gpio_num_t CAN_TX = GPIO_NUM_17;
constexpr gpio_num_t CAN_RX = GPIO_NUM_18;
constexpr uint32_t CAN_BITRATE = 1000000;
constexpr uint32_t MOTOR_ID = 1, FEEDBACK_ID = 0;
constexpr float POSITION_RANGE = 12.5f, VELOCITY_RANGE = 30.0f, TORQUE_RANGE = 10.0f;
struct Range { float lo, hi; };
// Position, velocity, feedforward torque, Kp, Kd. All start at zero.
constexpr Range SLIDERS[5] = {{-3.14f, 3.14f}, {-6.28f, 6.28f}, {-0.1f, 0.1f}, {0, 5}, {0, 0.2f}};
constexpr float STEPS[5] = {0.01f, 0.01f, 0.001f, 0.01f, 0.001f};
constexpr const char* LABELS[5] = {"Position rad", "Velocity rad/s", "Torque ff N m", "Kp", "Kd"};
constexpr int I2C_SDA = 2, I2C_SCL = 1; // CoreS3 Port A (yellow/white)
constexpr uint32_t COMMAND_MS = 10, FEEDBACK_TIMEOUT_MS = 500;
constexpr bool FLIP_180 = false;
}
