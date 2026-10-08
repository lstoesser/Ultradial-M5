#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

// Identical in both sketch folders. Explicit byte layout; no packed structs.
namespace telemetry {
constexpr uint8_t ADDRESS = 0x42;
constexpr uint32_t CLOCK_HZ = 100000;
constexpr size_t SIZE = 64;
constexpr uint8_t VERSION = 1;
enum Flags : uint8_t { Fresh = 1, Active = 2, Requested = 4, CanReady = 8 };
enum Reason : uint8_t { Starting, Stopped, Waiting, Running, InitFailed,
  BusOff, CanError, MotorFault, FeedbackLost, SendFailed, SpecialFailed };
inline const char* reasonName(uint8_t reason) {
  switch (reason) {
    case Starting: return "Starting"; case Stopped: return "Stopped";
    case Waiting: return "Waiting for motor"; case Running: return "Running";
    case InitFailed: return "CAN init failed"; case BusOff: return "CAN bus off";
    case CanError: return "CAN error"; case MotorFault: return "Motor fault";
    case FeedbackLost: return "Feedback lost"; case SendFailed: return "Send failed";
    case SpecialFailed: return "Special failed"; default: return "Unknown";
  }
}
struct Sample {
  uint32_t sequence = 0, sourceMs = 0, session = 0, rxCount = 0, errors = 0;
  uint16_t feedbackAge = 65535;
  uint8_t flags = 0, motorStatus = 255, reason = Starting;
  float measured[3] = {};
  float command[5] = {}; // position, velocity, feedforward torque, Kp, Kd
};
inline void put16(uint8_t* p, uint16_t n) { p[0] = n; p[1] = n >> 8; }
inline uint16_t get16(const uint8_t* p) { return uint16_t(p[0]) | (uint16_t(p[1]) << 8); }
inline void put32(uint8_t* p, uint32_t n) {
  for (int i = 0; i < 4; ++i) p[i] = n >> (8 * i);
}
inline uint32_t get32(const uint8_t* p) {
  uint32_t n = 0; for (int i = 0; i < 4; ++i) n |= uint32_t(p[i]) << (8 * i); return n;
}
inline void putFloat(uint8_t* p, float f) {
  static_assert(sizeof(float) == 4, "Requires IEEE float32");
  uint32_t bits; memcpy(&bits, &f, 4); put32(p, bits);
}
inline float getFloat(const uint8_t* p) {
  uint32_t bits = get32(p); float f; memcpy(&f, &bits, 4); return f;
}
inline uint16_t crc(const uint8_t* p, size_t length) {
  uint16_t value = 0xFFFF; // CRC-16/CCITT-FALSE
  for (size_t i = 0; i < length; ++i) {
    value ^= uint16_t(p[i]) << 8;
    for (int b = 0; b < 8; ++b) value = (value & 0x8000) ? (value << 1) ^ 0x1021 : value << 1;
  }
  return value;
}
inline void pack(const Sample& s, uint8_t* p) {
  memset(p, 0, SIZE);
  p[0] = 'G'; p[1] = '2'; p[2] = VERSION; p[3] = s.flags;
  put32(p + 4, s.sequence); put32(p + 8, s.sourceMs);
  put16(p + 12, s.feedbackAge); p[14] = s.motorStatus; p[15] = s.reason;
  for (int i = 0; i < 3; ++i) putFloat(p + 16 + 4 * i, s.measured[i]);
  for (int i = 0; i < 5; ++i) putFloat(p + 28 + 4 * i, s.command[i]);
  put32(p + 48, s.rxCount); put32(p + 52, s.errors); put32(p + 56, s.session);
  put16(p + 62, crc(p, 62));
}
inline bool unpack(const uint8_t* p, size_t size, Sample& out) {
  if (size != SIZE || p[0] != 'G' || p[1] != '2' || p[2] != VERSION ||
      get16(p + 62) != crc(p, 62) || (p[3] & 0xF0) || p[60] || p[61]) return false;
  Sample s;
  s.flags = p[3]; s.sequence = get32(p + 4); s.sourceMs = get32(p + 8);
  s.feedbackAge = get16(p + 12); s.motorStatus = p[14]; s.reason = p[15];
  for (int i = 0; i < 3; ++i) {
    s.measured[i] = getFloat(p + 16 + 4 * i); if (!isfinite(s.measured[i])) return false;
  }
  for (int i = 0; i < 5; ++i) {
    s.command[i] = getFloat(p + 28 + 4 * i); if (!isfinite(s.command[i])) return false;
  }
  s.rxCount = get32(p + 48); s.errors = get32(p + 52); s.session = get32(p + 56);
  out = s; return true;
}
// Duplicate cached responses must not keep a frozen controller looking alive.
struct Receiver {
  Sample latest;
  bool seen = false;
  uint32_t receivedAt = 0;
  bool accept(const uint8_t* p, size_t size, uint32_t now) {
    Sample next;
    if (!unpack(p, size, next)) return false;
    if (seen && next.session == latest.session && next.sequence == latest.sequence) return false;
    latest = next; receivedAt = now; seen = true; return true;
  }
  bool live(uint32_t now, uint32_t timeout) const {
    return seen && uint32_t(now - receivedAt) <= timeout;
  }
  bool feedbackFresh(uint32_t now, uint32_t linkTimeout, uint32_t feedbackTimeout) const {
    return live(now, linkTimeout) && (latest.flags & Fresh) &&
      uint32_t(latest.feedbackAge) + uint32_t(now - receivedAt) <= feedbackTimeout;
  }
};
}
