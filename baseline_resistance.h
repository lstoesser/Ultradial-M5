#pragma once
#include <math.h>

// Speed-dependent drag with a quiet zone and a filtered, slew-limited magnitude.
// Never retain braking torque through a stop or reversal of measured velocity.
struct BaselineResistance {
  float magnitude = 0.0f;
  float direction = 0.0f;
  void reset() { magnitude = 0.0f; direction = 0.0f; }

  float update(float velocity, float limit, float dt) {
    if (!isfinite(velocity) || limit <= 0.0f || dt <= 0.0f || dt > 0.020f ||
        fabsf(velocity) <= 0.15f) {
      reset();
      return 0.0f;
    }
    const float sign = copysignf(1.0f, velocity);
    if (direction != sign) {
      reset();
      direction = sign;
      return 0.0f;
    }
    const float speed = fabsf(velocity) - 0.15f;
    const float target = limit * tanhf(speed / 3.0f);
    float step = (target - magnitude) * dt / (0.030f + dt);
    const float maxStep = 0.10f * dt;  // Nm/s, including changes to the UI slider.
    step = fmaxf(-maxStep, fminf(maxStep, step));
    magnitude = fmaxf(0.0f, fminf(limit, magnitude + step));
    return -sign * magnitude;
  }
};
