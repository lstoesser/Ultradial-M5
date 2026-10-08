#include <M5Unified.h>
#include "dial_ui.h"
#include "odrive_config.h"

namespace ultradial::ui {
namespace {
enum class Page { Main, Detents, Forces, Can };
Page page = Page::Main;
bool hapticsOn = true;
bool springOn = false;
bool springTurnStopOn = false;
bool hardStopsOn = false;
bool inertiaOn = false;
bool odriveOnline = false;
bool odriveArmed = false;
bool displayDirty = true;
int values[6] = {24, 50, 23, 0, 50, 50};  // Detent quantity/strength/width, baseline %, spring %, inertia %.
int activeSlider = -1;
constexpr int kRowY[3] = {65, 111, 157};

bool inside(int x, int y, int left, int top, int width, int height) {
  return x >= left && x < left + width && y >= top && y < top + height;
}
void text(const char* value, int x, int y, uint32_t color = TFT_BLACK) {
  M5.Display.setTextColor(color, TFT_WHITE);
  M5.Display.setTextDatum(textdatum_t::middle_left);
  M5.Display.drawString(value, x, y);
}
void button(const char* label, int x, int y, int width, bool selected = false) {
  M5.Display.drawRoundRect(x, y, width, 30, 5, selected ? TFT_BLUE : TFT_DARKGREY);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.setTextColor(selected ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
  M5.Display.drawString(label, x + width / 2, y + 15);
}
void drawSwitch(int y, bool enabled) {
  M5.Display.fillRoundRect(172, y, 60, 30, 15, enabled ? TFT_BLUE : 0xE7E3E7);
  M5.Display.fillCircle(enabled ? 217 : 187, y + 15, 12, TFT_WHITE);
}
void drawSetting(int row) {
  const char* labels[] = {"Quantity", "Strength", "Width", "Baseline", "Spring", "Inertia"};
  int y = kRowY[row >= 3 ? row - 3 : row];
  M5.Display.fillRect(0, y - 17, 320, 35, TFT_WHITE);
  text(labels[row], 8, y);
  M5.Display.fillRoundRect(82, y - 3, 100, 6, 3, TFT_LIGHTGREY);
  M5.Display.fillCircle(82 + values[row], y, 7, TFT_BLUE);
  button("-", 192, y - 15, 30);
  char value[12];
  snprintf(value, sizeof(value), row == 0 ? "%d" : "%d%%", values[row]);
  M5.Display.setTextDatum(textdatum_t::middle_center);
  M5.Display.drawString(value, 253, y);
  button("+", 285, y - 15, 30);
}
void draw() {
  M5.Display.fillScreen(TFT_WHITE);
  M5.Display.setTextSize(1);
  button("Dial", 5, 5, 74, page == Page::Main);
  button("Detents", 84, 5, 74, page == Page::Detents);
  button("Forces", 163, 5, 74, page == Page::Forces);
  button("CAN", 242, 5, 73, page == Page::Can);
  if (page == Page::Main) {
    text("Spring", 30, 60); drawSwitch(45, springOn);
    text("Haptics", 30, 98); drawSwitch(83, hapticsOn);
    text("Hard Stops (+/-180)", 30, 136); drawSwitch(121, hardStopsOn);
    text("1-turn spring", 30, 174); drawSwitch(159, springTurnStopOn);
    const char* mode = !hapticsOn ? (values[3] > 0 ? "Baseline resistance enabled" : (inertiaOn && values[5] > 0 ? "Inertia coast enabled" : "Force feedback off")) : springOn ? "Spring return" :
                       values[0] == 0 ? "Free rotation" : "Endless detents";
    if (hapticsOn && hardStopsOn) mode = springOn ? "Spring + hard stops" : "Detents/free + hard stops";
    text(mode, 30, 201, TFT_DARKGREEN);
  } else if (page == Page::Detents) {
    for (int i = 0; i < 3; ++i) drawSetting(i);
    text("0 quantity = no detents", 8, 187, TFT_DARKGREY);
    text("Width = soft center (% of spacing)", 8, 202, TFT_DARKGREY);
  } else if (page == Page::Forces) {
    drawSetting(3); drawSetting(4); drawSetting(5);
    text("Inertia enabled", 8, 196);
    drawSwitch(181, inertiaOn);
  } else {
    char value[64];
    snprintf(value, sizeof(value), "Node ID: %u", ultradial::odrive::kAxisNodeId);
    text(value, 20, 65);
    snprintf(value, sizeof(value), "Bitrate: %lu bit/s", (unsigned long)ultradial::odrive::kCanBitrate);
    text(value, 20, 95);
    snprintf(value, sizeof(value), "TX: GPIO %d   RX: GPIO %d", ultradial::odrive::kCanTxPin, ultradial::odrive::kCanRxPin);
    text(value, 20, 125);
    text("Configure in odrive_config.h", 20, 165, TFT_DARKGREY);
    text("Arm the ODrive in its GUI", 20, 190, TFT_DARKGREY);
  }
  text(!odriveOnline ? "Awaiting ODrive CAN" : odriveArmed ? "ODrive armed" : "ODrive idle - arm in GUI",
       20, 226, odriveOnline && odriveArmed ? TFT_DARKGREEN : TFT_RED);
  displayDirty = false;
}
void changeValue(int row, int value) {
  value = constrain(value, 0, 100);
  if (values[row] == value) return;
  values[row] = value;
  if (((page == Page::Detents && row < 3) || (page == Page::Forces && row >= 3)) && !displayDirty) drawSetting(row);
  else displayDirty = true;
}
}  // namespace

void begin() {
  // setup() initializes M5 once before initializing this UI.
  M5.Display.setRotation(1);
  displayDirty = true;
  draw();
}
void update() {
  M5.update();
  const auto touch = M5.Touch.getDetail();
  if (!touch.isPressed()) activeSlider = -1;
  if (touch.wasPressed()) {
    if (inside(touch.x, touch.y, 5, 5, 310, 30)) {
      page = touch.x < 82 ? Page::Main : touch.x < 161 ? Page::Detents : touch.x < 240 ? Page::Forces : Page::Can;
      activeSlider = -1;
      displayDirty = true;

    } else if (page == Page::Main) {
      if (inside(touch.x, touch.y, 172, 45, 60, 30)) { springOn = !springOn; displayDirty = true; }
      else if (inside(touch.x, touch.y, 172, 83, 60, 30)) { hapticsOn = !hapticsOn; displayDirty = true; }
      else if (inside(touch.x, touch.y, 172, 121, 60, 30)) { hardStopsOn = !hardStopsOn; displayDirty = true; }
      else if (inside(touch.x, touch.y, 172, 159, 60, 30)) { springTurnStopOn = !springTurnStopOn; displayDirty = true; }
    } else if (page == Page::Forces && inside(touch.x, touch.y, 172, 181, 60, 30)) {
      inertiaOn = !inertiaOn;
      displayDirty = true;
    } else if (page == Page::Detents || page == Page::Forces) {
      const int first = page == Page::Forces ? 3 : 0;
      const int end = page == Page::Forces ? 6 : 3;
      for (int row = first; row < end; ++row) {
        int y = kRowY[row - first] - 15;
        if (inside(touch.x, touch.y, 75, y, 114, 30)) activeSlider = row;
        else if (inside(touch.x, touch.y, 192, y, 30, 30)) changeValue(row, values[row] - 1);
        else if (inside(touch.x, touch.y, 285, y, 30, 30)) changeValue(row, values[row] + 1);
      }
    }
  }
  if (activeSlider >= 0 && touch.isPressed()) changeValue(activeSlider, touch.x - 82);
  if (displayDirty) draw();
}
void setOdriveStatus(bool online, bool armed) {
  if (odriveOnline != online || odriveArmed != armed) {
    odriveOnline = online; odriveArmed = armed; displayDirty = true;
  }
}
bool hapticsEnabled() { return hapticsOn; }
bool springEnabled() { return springOn; }
bool springTurnStopEnabled() { return springTurnStopOn; }
bool hardStopsEnabled() { return hardStopsOn; }
bool inertiaEnabled() { return inertiaOn; }
int detentQuantity() { return values[0]; }
float detentStrength() { return values[1] / 100.0f; }
float detentWidth() { return values[2] / 100.0f; }
float baselineResistance() { return values[3] / 100.0f; }
float springStrength() { return values[4] / 100.0f; }
void setSpringEnabled(bool enabled) { if (springOn != enabled) { springOn = enabled; displayDirty = true; } }
void setDetentQuantity(int quantity) { changeValue(0, quantity); }
}  // namespace ultradial::ui


namespace ultradial::ui {
float inertiaStrength() { return values[5] / 100.0f; }
}
