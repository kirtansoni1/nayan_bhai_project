#pragma once

#include <Arduino.h>

// -------------------- Pin mapping --------------------
// Stepper (DM542)
constexpr gpio_num_t PIN_S_M1_STEP = GPIO_NUM_1;
constexpr gpio_num_t PIN_S_M1_DIR = GPIO_NUM_2;
constexpr gpio_num_t PIN_S_M2_STEP = GPIO_NUM_4;
constexpr gpio_num_t PIN_S_M2_DIR = GPIO_NUM_5;
constexpr gpio_num_t PIN_S_M3_STEP = GPIO_NUM_6;
constexpr gpio_num_t PIN_S_M3_DIR = GPIO_NUM_7;
constexpr gpio_num_t PIN_S_M_EN = GPIO_NUM_40;

// DC 3000 RPM (two BTS7960 drivers in parallel control)
constexpr gpio_num_t PIN_DC_3000_RPWM = GPIO_NUM_10;
constexpr gpio_num_t PIN_DC_3000_LPWM = GPIO_NUM_11;
constexpr gpio_num_t PIN_DC_3000_EN = GPIO_NUM_42;

// DC 300 RPM (independent BTS7960 control)
constexpr gpio_num_t PIN_DC1_300_RPWM = GPIO_NUM_8;
constexpr gpio_num_t PIN_DC1_300_LPWM = GPIO_NUM_9;
constexpr gpio_num_t PIN_DC2_300_RPWM = GPIO_NUM_48;
constexpr gpio_num_t PIN_DC2_300_LPWM = GPIO_NUM_47;
constexpr gpio_num_t PIN_DC_300_EN = GPIO_NUM_41;

// 4x4 button matrix (8 wires total)
// Rows
constexpr gpio_num_t PIN_BTN_R0 = GPIO_NUM_12;
constexpr gpio_num_t PIN_BTN_R1 = GPIO_NUM_13;
constexpr gpio_num_t PIN_BTN_R2 = GPIO_NUM_14;
constexpr gpio_num_t PIN_BTN_R3 = GPIO_NUM_15;
// Cols
constexpr gpio_num_t PIN_BTN_C0 = GPIO_NUM_16;
constexpr gpio_num_t PIN_BTN_C1 = GPIO_NUM_17;
constexpr gpio_num_t PIN_BTN_C2 = GPIO_NUM_18;
constexpr gpio_num_t PIN_BTN_C3 = GPIO_NUM_21;

// Solenoid relay
constexpr gpio_num_t PIN_SOLENOID_RLY = GPIO_NUM_39;

// -------------------- Shared constants --------------------
constexpr uint8_t STEPPER_MOTOR_COUNT = 3;

// Stepper defaults
constexpr float STEPPER_DEFAULT_MAX_SPEED = 1200.0f; // In steps per second
constexpr float STEPPER_DEFAULT_ACCEL = 800.0f; // In steps per second squared
constexpr float STEPPER_DEFAULT_DECEL = 800.0f; // In steps per second squared

// DC PWM defaults
constexpr uint32_t DC_PWM_FREQ_HZ = 20000; // In Hertz
constexpr uint8_t DC_PWM_BITS = 8; // Resolution in bits
constexpr uint8_t DC_PWM_MAX = 255; // Maximum PWM value

// Key scan timing
constexpr uint32_t KEY_SCAN_PERIOD_MS = 2; // In milliseconds
constexpr uint32_t KEY_DEBOUNCE_MS = 20; // In milliseconds

enum class Direction : uint8_t {
  CW = 0,
  CCW = 1,
};

enum class SolenoidState : uint8_t {
  OFF = 0,
  ON = 1,
};
