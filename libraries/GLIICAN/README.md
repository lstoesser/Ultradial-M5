# GLIICAN (ESP32)

Version 0.1.1 adds `readBusEvents()` (bus-off, TX failure, RX overflow) and
`discardPendingCommands()` so applications can handle faults and clear queued
commands without calling TWAI directly. Both must be called from the same task
as the other library methods. Queue clearing cannot cancel a frame already on
the wire and does not itself disable the motor.

Install this GLIICAN folder in your Arduino sketchbook's libraries directory, then open File > Examples > GLIICAN > Diagnostics. This uses the legacy driver/twai.h API, tested with Arduino ESP32 2.0.16 on CoreS3.

All project choices are in glii::Config: pins, bitrate, command IDs, reply ID/type, single-shot TX, and encoding ranges. Construct GLIICAN from the config, register onFeedback once, call begin once, and call update continuously. One instance owns the global TWAI controller; do not install another TWAI driver or separately drain its receive queue.

The sample retains motor ID 1, 1 Mbit/s, and ranges 12.5/30/10 from your source. Reply ID 1 MUST be verified. Earlier notes said velocity range 200; both transmit and receive scaling must match the actual motor configuration. Velocity units also require confirmation. Position/velocity and velocity command IDs are explicit because mode-specific addressing has not been established; the library does not switch the board's saved control mode.

Commands return whether the CAN frame was queued, not whether the motor executed it. begin() does not enable the motor. The example accepts e/x/z/c serial commands for enable/disable/zero/clear errors and sends zero-gain, zero-torque MIT packets while explicitly enabled. Startup waits for valid feedback but still accepts these serial commands. Feedback presence does not imply a healthy or enabled motor.

Request tracking matches full reply ID, standard/extended type and length. Without a protocol sequence field, unsolicited telemetry or late replies with the same signature can satisfy a pending request. Call from a single task, not an ISR; callbacks receive temporary references and should copy data they retain.

## Why the old files did not compile

- gl-ii-CAN.cpp did not include gl-ii-CAN.h. Each .cpp is compiled independently; an include in the .ino does not carry over.
- Definitions such as special(), mit(), update() were free functions, not GLIICAN::special(), GLIICAN::mit(), GLIICAN::update(). They could not access class members.
- The free sendMsg required an ESP32TWAIIntf argument, but callers passed only ID, length and data. The refactor uses a private GLIICAN::sendMsg member instead.
- CanIntfWrapper vs GLIICanIntfWrapper, and context/send vs can_intf_/send_msg_, mixed different versions of the wrapper.
- len(data) is not C++; use sizeof(data) for a local fixed array.
- The header used TWAI types without including driver/twai.h, and defined non-inline pumpEvents in the header (multiple-definition risk).
- The .cpp had duplicate structures, a file-scope private: block and stray class closing brace, and an incomplete && expression.
- enabled/lastCommand/lastStatus lived in another translation unit without declarations; they belong in the application sketch.
- The sketch's direct twai_receive loop consumed frames instead of delivering them to the driver's callbacks.

The library now declares its API in src/gl-ii-CAN.h and defines it once in src/gl-ii-CAN.cpp. It also fixes unsigned packed position decoding, signed temperatures, and the receive-loop bound check.
