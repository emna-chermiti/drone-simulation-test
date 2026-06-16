// compat/Arduino.h
// Replaces the entire ESP32 Arduino framework for Windows compilation
#pragma once
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdarg>
#include <algorithm>

using std::max;
using std::min;

inline unsigned long micros() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<microseconds>(
        steady_clock::now().time_since_epoch()).count();
}
inline unsigned long millis() {
    using namespace std::chrono;
    return (unsigned long)duration_cast<milliseconds>(
        steady_clock::now().time_since_epoch()).count();
}
inline void delay(unsigned long ms)            { (void)ms; }
inline void delayMicroseconds(unsigned int us) { (void)us; }

template<typename T>
inline T constrain(T x, T lo, T hi) { return x<lo?lo:(x>hi?hi:x); }

template<typename T>
inline T map(T x, T iL, T iH, T oL, T oH) {
    return (x-iL)*(oH-oL)/(iH-iL)+oL;
}

#ifndef PI
  #define PI 3.14159265358979f
#endif
#define DEG_TO_RAD (PI/180.0f)
#define RAD_TO_DEG (180.0f/PI)
#define TWO_PI     (2.0f*PI)

inline void ledcAttach(int pin, int freq, int res) {
    (void)pin; (void)freq; (void)res;
}
inline void ledcWrite(int pin, int duty) { (void)pin; (void)duty; }

struct HardwareSerial {
    void begin(int baud)        { (void)baud; }
    void println(const char* s) {
    if (strstr(s, "[PID]") == nullptr)
        printf("[ESP] %s\n", s);}
    void print(const char* s)   { printf("%s", s); }
    void printf(const char* fmt, ...) {
        va_list a; va_start(a,fmt); vprintf(fmt,a); va_end(a);
    }
    bool available()            { return false; }
    char read()                 { return 0; }
};
extern HardwareSerial Serial;

typedef uint8_t  byte;
typedef bool     boolean;

#ifndef MOTOR_MIN
  #define MOTOR_MIN        30
#endif
#ifndef MOTOR_MAX
  #define MOTOR_MAX       200
#endif
#ifndef MOTOR_ARM_TARGET
  #define MOTOR_ARM_TARGET 40
#endif
#ifndef PWM_FREQ
  #define PWM_FREQ       20000
#endif
#ifndef PWM_RES
  #define PWM_RES            8
#endif
#ifndef PWM_MAX_DUTY
  #define PWM_MAX_DUTY     255
#endif