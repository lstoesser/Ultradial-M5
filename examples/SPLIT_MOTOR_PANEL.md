# Split motor tuner: CoreS3 control + Tab5 graphs

Upload a different Arduino sketch to each device:

- CoreS3: `examples/CoreS3_Motor_Controller/CoreS3_Motor_Controller.ino`
- Tab5: `examples/Tab5_Motor_Panel/Tab5_Motor_Panel.ino`

Keep the .h files in each sketch folder alongside the .ino. The old Tab5 motor
panel is now a display-only program; it no longer initializes CAN or accepts
motor commands. Move the PwrCAN module to the CoreS3.

## Hardware wiring

Power each device separately. Use a three-wire connection between their Port A
connectors. Disconnect/omit the red 5V wire; do not join two powered 5V outputs.

| Signal | CoreS3 Port A | Tab5 Port A |
| --- | --- | --- |
| SDA (yellow) | GPIO2 | GPIO53 |
| SCL (white) | GPIO1 | GPIO54 |
| GND (black) | GND | GND |
| 5V (red) | Leave disconnected | Leave disconnected |

Keep the I2C cable short (start around 10-20 cm). SDA and SCL need pull-ups to
3.3V, not 5V. If the bus does not already have pull-ups, add a 4.7 kOhm resistor
from SDA to 3.3V and another from SCL to 3.3V. Do not blindly add extra pull-ups
if they are already present. Verify signal labels rather than relying only on
wire colors. This is the external Port A bus, not the internal touch/power bus.

PwrCAN connects to the CoreS3 with CAN TX=GPIO17, RX=GPIO18, using M5-Bus
pins 16/15. Retain the working motor power, CAN H/L wiring and termination.
Motor CAN defaults remain 1 Mbps, motor ID 1, reply/Master ID 0.

## Arduino IDE

Install M5Unified and M5GFX for both devices (tested APIs: 0.2.25 / 0.2.32).
The CoreS3 additionally requires GLIICAN 0.1.1 (`gl-ii-CAN.h`), from the library
ZIP previously supplied. No further GLIICAN update is required for this split.
Tab5 does not require GLIICAN.

Open each .ino in its own Arduino IDE window, select that device's board and
COM port, compile and upload. Enable PSRAM for Tab5. No PlatformIO setup is
required for use. The local platformio.split-panel.ini is only a build-check
configuration: its Tab5 environment targets ESP32-S3 APIs, not a Tab5 binary.
Do not upload that API-check binary to Tab5.

## CoreS3 controls

All five sliders are on the 320x240 screen: position, velocity, feedforward
torque, Kp and Kd. Touch/drag a track to change a value. Touch a row to select
it (cyan label), then use the bottom minus/plus buttons for one configured
step at a time. RUN starts MIT commands at 100 Hz; STOP requests motor exit.
Values may be prepared while stopped and are applied live while running.

Edit `CoreS3_Motor_Controller/ControlConfig.h` for CAN IDs/pins/bitrate, motor
encoding ranges, slider limits, rounding steps and screen orientation.
The motor must already be configured for MIT mode. These sliders adjust the
MIT command gains, not persistent internal motor-controller parameters.

For velocity tuning, use Kp=0 and zero feedforward torque, select velocity, and
adjust Kd. For position tuning, set desired velocity to zero and adjust Kp/Kd
while changing position. The torque slider is feedforward; total feedback
torque also includes the position/velocity controller's contribution.

Serial shortcuts at 115200 remain: e=Run, x=Stop, z=set zero, c=clear fault.
Zero and fault reset are accepted only while stopped. All CAN operations,
including serial requests, run through GLIICAN in the CoreS3 CAN task.
The controller starts stopped and does not auto-resume after faults or restart.
No enabled feedback within 500 ms, later stale feedback, motor faults and CAN
errors stop streaming and attempt a disable command. A failed CAN link cannot
guarantee that disable reaches the motor; retain the motor's communication timeout.

## Tab5 display

Three full-width plots show measured position, velocity and torque in color.
Dashed white lines show the commanded values while the CoreS3 is streaming.
The plots retain about 10 seconds, sample at 20 Hz, and draw at 10 Hz.
Kp/Kd and connection/motor status are read-only in the header. There are no
sliders or control buttons on Tab5. Its previous 180-degree flip is retained.

Edit `Tab5_Motor_Panel/PanelConfig.h` for graph limits, units, timing and
FLIP_180. Values outside the plotted range are clipped and labeled OFF SCALE;
the numerical value remains visible. Confirm velocity units for your motor.

## I2C behavior

Tab5 is the I2C master; CoreS3 is a read-only slave at address 0x42, 100 kHz.
Being I2C master does not give Tab5 motor-control authority. It only polls a
cached telemetry packet; no motor commands are implemented over this link.
CoreS3 uses external I2C0; internal I2C1 remains available for touchscreen and
power management. Tab5 uses M5Unified's external I2C controller.

The CAN task publishes a 64-byte snapshot with version, boot session, sequence,
feedback age, motor/control status, three measured values, five requested
values, counters, and CRC-16/CCITT-FALSE. The I2C callback only copies/sends
that cache and never calls the motor library or draws the screen.
`Telemetry.h` is intentionally identical in both sketch folders so each sketch
can be copied independently. Keep both copies synchronized if changing the protocol.

Disconnecting Tab5 does not change motor control: use CoreS3 STOP to stop.
CoreS3 indicates Tab offline; Tab5 marks the link offline after 300 ms without
an advancing valid packet. Invalid frames and duplicate cached frames do not
refresh the timer. Missing/stale feedback produces gaps and -- numeric readings;
it is never plotted as zero. A CoreS3 reboot clears the graph history when its
new boot session arrives. Reconnecting Tab5 resumes plotting automatically.

## First test

1. Upload both sketches. Leave the motor stopped.
2. Connect the three I2C wires and verify CoreS3 says Tab:linked and Tab5 shows
   controller status. No motor is required to test this telemetry link.
3. Change a gain on CoreS3 while stopped: the read-only Kp/Kd values on Tab5
   should follow. If offline, check SDA/SCL, ground, pull-ups and that the
   correct sketch/board is loaded on each device.
4. With the motor connected in MIT mode, verify FEEDBACK_ID against the working
   CAN setup, then use RUN and observe feedback. Use the CoreS3 STOP button.
5. Disconnect/reconnect the display link while stopped and confirm Tab5 stops
   presenting old data as live and automatically reconnects.

## Verification

The CoreS3 firmware and Tab5 graph API check are compiled locally. Physical
I2C/touch/CAN operation and an ESP32-P4 Tab5 build still require your devices.

References:
- CoreS3 pinout: https://docs.m5stack.com/en/core/CoreS3
- Tab5 pinout: https://docs.m5stack.com/en/core/Tab5
- ESP32 Arduino I2C: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html
