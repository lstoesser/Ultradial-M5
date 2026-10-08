# CoreS3 USB-to-UART bridge

This exposes a USB CDC COM port and forwards bytes to UART1 on GPIO17/18.
It does not emulate S-Link's USB identity or any S-Link-specific commands.
CubeMars software compatibility remains unverified: a program that requires
S-Link rather than a plain serial port will need the real adapter or its protocol.

## Wiring

Remove the PwrCAN stack for this test, or disconnect every CAN/RS485 DIP route
to GPIO17/18. A transceiver output must not share the UART receive wire.

| CoreS3 Port C / M5-Bus | Motor's separate 3-pin UART connector |
| --- | --- |
| GPIO17 TX (M5-Bus pin 16) | RX (manual pin 3) |
| GPIO18 RX (M5-Bus pin 15) | TX (manual pin 2) |
| GND | GND (manual pin 1) |

Leave the Port C 5V wire disconnected. Power the motor separately through its
normal power connector. Connector numbering is from the motor manual; check
orientation rather than assuming cable colors or left-to-right order.

The CoreS3 uses 3.3V logic and is not 5V tolerant. The GL II documentation checked
identifies GND/TX/RX but does not establish their logic voltage. Verify the motor
UART levels before direct connection; use a suitable level translator if needed.
This is TTL UART, not RS232 and not CAN.

## Upload with Arduino IDE

1. Open CoreS3_UART_Bridge.ino and select M5Stack CoreS3.
2. Set USB Mode to USB-OTG (TinyUSB), USB CDC On Boot to Enabled, and
   Core Debug Level to None. Board menu wording depends on the installed package.
3. Upload over USB-C. The USB COM port may change after flashing.
4. If upload no longer connects, hold the CoreS3 reset button until its green
   download-mode indication appears, then select its download COM port and upload.

No third-party libraries are required. The screen is not initialized.

PlatformIO, from the ultradial_m5 directory:

```
pio run -c platformio.bridge.ini
pio run -c platformio.bridge.ini -t upload
```

## Check the bridge before connecting the motor

1. With the motor disconnected, connect GPIO17 to GPIO18.
2. Open the new USB COM port at 115200, 8N1, no flow control, DTR enabled.
3. Send text with local echo disabled. The same text should return.
4. Close the port and remove the loopback wire before wiring the motor.

## Use with the motor

Open the COM port before powering the motor to capture its startup output.
115200 is a starting guess, not a confirmed motor baud rate. By default the UART
follows the baud requested by the PC application; choose the motor's actual baud
there. Set FOLLOW_HOST_BAUD=false and DEFAULT_BAUD as needed for a fixed rate.
Only 8N1 framing is supported; PC parity/stop-bit settings are not forwarded.
DTR/RTS do not control motor wires or reset the CoreS3. The USB CDC stack may
require the PC application to assert DTR to receive data.

Close Serial Monitor before opening CubeMars software so only one application
owns the COM port. Try reading parameters first. This bridge does not itself
write parameters or issue motor commands. If boot text is readable but CubeMars
cannot read parameters, its adapter protocol may require S-Link; do not assume
the bridge is a compatible replacement. No CAN interface is provided by this sketch.

Buffers are finite; sustained motor output while the PC is disconnected or not
reading can overflow. Power-cycle the motor with the COM port open for a fresh
startup capture. Do not use this unverified bridge for firmware flashing.

References:
- https://docs.m5stack.com/en/core/CoreS3
- https://www.cubemars.com/images/file/20241231/1735633965678815.pdf
