#include <Arduino.h>
#include "USB.h"
#include "USBCDC.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Select an ESP32-S3 board (M5Stack CoreS3)."
#endif
#if ARDUINO_USB_MODE
#error "Select USB Mode: USB-OTG (TinyUSB), not Hardware CDC and JTAG."
#endif

#if ARDUINO_USB_CDC_ON_BOOT
#define HostUSB Serial
#else
USBCDC HostUSB;
#endif

// CoreS3 Port C / M5-Bus pins 16 (TX) and 15 (RX).
// Disconnect PwrCAN/RS485 DIP routes from these pins before use.
constexpr int MOTOR_RX_PIN = 18; // Motor TX -> CoreS3 RX
constexpr int MOTOR_TX_PIN = 17; // CoreS3 TX -> motor RX
constexpr uint32_t DEFAULT_BAUD = 115200; // Starting guess, not a verified GL II setting.
constexpr bool FOLLOW_HOST_BAUD = true;
// UART is always 8 data bits, no parity, 1 stop bit; no hardware flow control.
HardwareSerial MotorUART(1);
uint32_t activeBaud = DEFAULT_BAUD;

struct PendingBytes {
  uint8_t bytes[256];
  size_t offset = 0;
  size_t count = 0;
};
PendingBytes toMotor, toHost;

// Keep unwritten bytes for the next loop; do not drop partial writes.
void forwardBytes(Stream &source, Stream &destination, PendingBytes &pending);
void forwardBytes(Stream &source, Stream &destination, PendingBytes &pending) {
  if (pending.offset == pending.count) {
    pending.offset = pending.count = 0;
    int available = source.available();
    if (available > 0) {
      size_t count = min((size_t)available, sizeof(pending.bytes));
      for (size_t i = 0; i < count; ++i) {
        int value = source.read();
        if (value < 0) break;
        pending.bytes[pending.count++] = (uint8_t)value;
      }
    }
  }
  int room = destination.availableForWrite();
  if (room > 0 && pending.offset < pending.count) {
    size_t count = min((size_t)room, pending.count - pending.offset);
    pending.offset += destination.write(pending.bytes + pending.offset, count);
  }
}

void setup() {
  MotorUART.setRxBufferSize(4096);
  MotorUART.begin(activeBaud, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN);
  HostUSB.setRxBufferSize(4096);
  HostUSB.setTxTimeoutMs(0);
  // PC applications may toggle DTR/RTS or select 1200 baud. Do not reboot.
  HostUSB.enableReboot(false);
  HostUSB.begin();
  USB.begin();
  // Never print debug/status text to HostUSB: this is a binary-transparent link.
}

void loop() {
  uint32_t requestedBaud = FOLLOW_HOST_BAUD ? HostUSB.baudRate() : DEFAULT_BAUD;
  if (requestedBaud != 0 && requestedBaud != activeBaud) {
    MotorUART.flush();
    MotorUART.updateBaudRate(requestedBaud);
    activeBaud = requestedBaud;
  }
  forwardBytes(HostUSB, MotorUART, toMotor);
  forwardBytes(MotorUART, HostUSB, toHost);
  delay(1);
}
