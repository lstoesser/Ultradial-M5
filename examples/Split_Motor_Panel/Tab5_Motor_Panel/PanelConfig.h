#pragma once
#include <stdint.h>
namespace config {
// Tab5 only displays telemetry. All motor settings live on CoreS3.
constexpr int I2C_SDA = 53, I2C_SCL = 54;
constexpr bool FLIP_180 = true;
struct Range { float lo, hi; };
constexpr Range PLOTS[3] = {{-3.14f, 3.14f}, {-6.28f, 6.28f}, {-0.10f, 0.10f}};
constexpr const char* UNITS[3] = {"rad", "rad/s", "N m"};
constexpr uint32_t POLL_MS = 50, PLOT_MS = 50, DRAW_MS = 100;
constexpr uint32_t LINK_TIMEOUT_MS = 300, FEEDBACK_TIMEOUT_MS = 500;
constexpr int SAMPLES = 200;
}
