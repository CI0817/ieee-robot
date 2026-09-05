#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
using std::abs;
using std::max;
template<class T> T constrain(T x, T lo, T hi) { return std::min(std::max(x, lo), hi); }
constexpr int INPUT = 0, OUTPUT = 1, HIGH = 1, LOW = 0;
extern uint32_t test_ms;
extern int readings[40];
inline uint32_t millis() { return test_ms; }
inline int analogRead(int pin) { return readings[pin]; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline void ledcSetup(int, int, int) {}
inline void ledcAttachPin(int, int) {}
inline void ledcWrite(int, int) {}
