#pragma once
#include <algorithm>
#include <cstdint>
#include <cstdlib>
constexpr int INPUT = 0, OUTPUT = 1, LOW = 0, HIGH = 1;
template <typename T> T constrain(T x, T lo, T hi) {
  return std::min(std::max(x, lo), hi);
}
extern unsigned long fake_ms;
extern int fake_ir[40];
extern int read_count;
inline unsigned long millis() { return fake_ms; }
inline int analogRead(int pin) { ++read_count; return fake_ir[pin]; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int, int) {}
struct SerialStub { void begin(int) {} };
static SerialStub Serial;
