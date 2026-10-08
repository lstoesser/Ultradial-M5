#pragma once
#include <math.h>

// A decaying velocity envelope, not a position target. No release sensor needed.
struct CoastAssist {
  float speed = 0.0f;
  void reset() { speed = 0.0f; }

  float update(float velocity, float amount, float dt, float torqueLimit) {
    if (amount <= 0.0f || dt <= 0.0f || dt > 0.020f ||
        fabsf(velocity) < 0.2f || speed * velocity < 0.0f) {
      reset();
      return 0.0f;
    }
    // Higher settings retain momentum longer (up to a 1.2-second decay time).
    speed *= expf(-dt / (0.15f + 1.05f * amount));
    // Only a deliberate fast spin (> approximately 0.5 rev/sec) starts a coast.
    if (fabsf(velocity) > 3.0f && fabsf(velocity) > fabsf(speed)) {
      speed = velocity;
    }
    const float difference = fabsf(speed) - fabsf(velocity);
    if (difference <= 0.0f) return 0.0f;
    // Fade in the assist and never command more than 20% of the motor limit.
    return copysignf(0.20f * torqueLimit * amount * tanhf(difference / 2.0f), speed);
  }
};
