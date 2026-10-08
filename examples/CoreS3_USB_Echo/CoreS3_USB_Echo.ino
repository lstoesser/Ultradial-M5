#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select M5Stack CoreS3 / ESP32-S3."
#endif
#if ARDUINO_USB_MODE
#error "Select USB Mode: USB-OTG (TinyUSB)."
#endif

#if ARDUINO_USB_CDC_ON_BOOT
#define TestUSB Serial
#else
USBCDC TestUSB;
#endif

// USB-only diagnostic. No UART pins or motor connections are used.
uint32_t lastBanner = 0;
int pendingByte = -1;

void setup() {
  TestUSB.enableReboot(false);
  TestUSB.setTxTimeoutMs(10);
  TestUSB.begin();
  USB.begin();
}

void loop() {
  const char banner[] = "\r\nCORES3 USB ECHO TEST - firmware running\r\n";
  if (millis() - lastBanner >= 2000 &&
      TestUSB.availableForWrite() >= (int)(sizeof(banner) - 1)) {
    TestUSB.write((const uint8_t *)banner, sizeof(banner) - 1);
    lastBanner = millis();
  }
  if (pendingByte < 0 && TestUSB.available()) pendingByte = TestUSB.read();
  if (pendingByte >= 0 && TestUSB.availableForWrite() > 0) {
    if (TestUSB.write((uint8_t)pendingByte) == 1) pendingByte = -1;
  }
  delay(1);
}
