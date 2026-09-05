#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
using std::min;
using std::max;
constexpr int HIGH=1, LOW=0, INPUT=0, OUTPUT=1;
template<class T> T constrain(T x,T lo,T hi) { return min(max(x,lo),hi); }
extern int mockRaw[40];
extern uint32_t mockMicros;
inline int analogRead(int pin) { return mockRaw[pin]; }
inline void analogReadResolution(int) {}
inline void pinMode(int,int) {}
inline void digitalWrite(int,int) {}
inline void analogWrite(int,int) {}
inline uint32_t micros() { return mockMicros; }
inline uint32_t millis() { return mockMicros/1000; }
struct MockSerial {
 void begin(int) {}
 void println(const char*) {}
 int available() { return 0; }
 int read() { return -1; }
 int availableForWrite() { return 128; }
 template<class... Args> void printf(const char*,Args...) {}
};
extern MockSerial Serial;
