#pragma once

#include <Arduino.h>

// Serial logging. Build with -DLOG_ENABLED=0 to strip all prints (production).
#ifndef LOG_ENABLED
#define LOG_ENABLED 1
#endif

#if LOG_ENABLED
#define LOG_BEGIN(baud) \
  do {                  \
    Serial.begin(baud); \
    delay(500);         \
  } while (0)
#define LOG(...) Serial.printf(__VA_ARGS__)
#define LOGLN(msg) Serial.println(msg)
#else
// "if (0)" keeps arguments type checked and used, compiler removes the call
#define LOG_BEGIN(baud) \
  do {                  \
    (void)(baud);       \
  } while (0)
#define LOG(...)                             \
  do {                                       \
    if (0) Serial.printf(__VA_ARGS__);       \
  } while (0)
#define LOGLN(msg)                           \
  do {                                       \
    if (0) Serial.println(msg);              \
  } while (0)
#endif
