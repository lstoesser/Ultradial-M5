#pragma once

// Touch UI for the CoreS3-Lite.  Its layout mirrors M5 UI/Ultradial.m5f2.
// Call begin() once, then update() on every pass through loop().
namespace ultradial::ui {

void begin();
void update();
void setOdriveStatus(bool online, bool armed);
bool hapticsEnabled();
bool springEnabled();
bool springTurnStopEnabled();
bool hardStopsEnabled();
int detentQuantity();
float detentStrength();  // 0..1; 0.5 preserves the original stiffness.
float detentWidth();     // Soft center as a fraction of detent spacing.
float baselineResistance();  // 0..1, independent of the Haptics switch.
float springStrength();      // 0..1; 0.5 preserves the original stiffness.
float inertiaStrength();    // 0..1; zero disables coast assist.
bool inertiaEnabled();
void setSpringEnabled(bool enabled);
void setDetentQuantity(int quantity);

}  // namespace ultradial::ui
