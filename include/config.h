#pragma once

#include <Arduino.h>

// Board configuration. Every pin, polarity and tuning value lives here.

enum class Direction : uint8_t {
  CW = 0,
  CCW = 1,
};

// -------------------- I2C / TCA9535 expander --------------------
constexpr uint8_t PIN_I2C_SDA = 10;
constexpr uint8_t PIN_I2C_SCL = 11;
constexpr uint32_t I2C_FREQ_HZ = 100000;
constexpr uint8_t IO_EXPANDER_ADDR = 0x20;  // A2..A0 tied low

// Expander pin numbers (0..7 = P00..P07, 8..15 = P10..P17). EXIOn = P0n.
constexpr uint8_t EXIO_SOLENOID_RLY = 1;
constexpr uint8_t EXIO_DC_3000_EN = 2;
constexpr uint8_t EXIO_DC1_300_EN = 3;
constexpr uint8_t EXIO_DC2_300_EN = 4;
constexpr uint8_t EXIO_S_M1_EN = 5;
constexpr uint8_t EXIO_S_M2_EN = 6;
constexpr uint8_t EXIO_S_M3_EN = 7;

// -------------------- DC motors (BTN7960B) --------------------
constexpr uint8_t PIN_DC1_300_RPWM = 12;
constexpr uint8_t PIN_DC1_300_LPWM = 13;
constexpr uint8_t PIN_DC2_300_RPWM = 38;
constexpr uint8_t PIN_DC2_300_LPWM = 47;
constexpr uint8_t PIN_DC_3000_RPWM = 14;
constexpr uint8_t PIN_DC_3000_LPWM = 21;

constexpr bool DC_EN_ACTIVE_HIGH = true;     // BTN7960B INH: high = driver on
constexpr uint32_t DC_PWM_FREQ_HZ = 20000;   // BTN7960B max is 25 kHz
constexpr uint8_t DC_PWM_BITS = 8;
constexpr uint8_t DC_PWM_MAX = 255;

// Swap motor direction without rewiring (true = CW drives LPWM instead of RPWM)
constexpr bool DC1_300_DIR_INVERT = false;
constexpr bool DC2_300_DIR_INVERT = false;
constexpr bool DC_3000_DIR_INVERT = false;

// -------------------- Stepper motors (DM542) --------------------
constexpr uint8_t STEPPER_COUNT = 3;

constexpr uint8_t PIN_S_M1_STEP = 1;
constexpr uint8_t PIN_S_M1_DIR = 2;
constexpr uint8_t PIN_S_M2_STEP = 42;
constexpr uint8_t PIN_S_M2_DIR = 41;
constexpr uint8_t PIN_S_M3_STEP = 40;
constexpr uint8_t PIN_S_M3_DIR = 39;

constexpr bool STEPPER_EN_ACTIVE_HIGH = false;  // LOW on EN = driver enabled

// Swap stepper direction without rewiring
constexpr bool S_M1_DIR_INVERT = false;
constexpr bool S_M2_DIR_INVERT = false;
constexpr bool S_M3_DIR_INVERT = false;

// DM542 timing: PUL width >= 2.5 us, DIR must lead PUL by >= 5 us
constexpr uint16_t STEPPER_PULSE_WIDTH_US = 5;
constexpr uint16_t STEPPER_DIR_SETUP_US = 10;

constexpr float STEPPER_MAX_SPEED = 12000.0f;  // steps/s
constexpr float STEPPER_ACCEL = 8000.0f;       // steps/s^2, same rate used to decelerate

// Deceleration used when STOP or the sensor halts a move (steps/s^2).
// 0 = freeze instantly (can lose steps at speed, position tracking may drift).
constexpr float STEPPER_HALT_DECEL = 32000.0f;

// -------------------- Solenoid relay --------------------
constexpr bool SOLENOID_ACTIVE_HIGH = true;

// -------------------- LEDs --------------------
constexpr uint8_t PIN_LED_GREEN = 19;
constexpr uint8_t PIN_LED_RED = 20;
constexpr bool LED_ACTIVE_HIGH = true;

// Error blink code on RED: N blinks, pause, repeat (N = error number)
constexpr uint32_t ERR_BLINK_ON_MS = 250;
constexpr uint32_t ERR_BLINK_OFF_MS = 250;
constexpr uint32_t ERR_BLINK_PAUSE_MS = 1500;

// I2C write attempts before the expander is declared faulty
constexpr uint8_t IO_EXPANDER_WRITE_TRIES = 3;

// -------------------- Buttons and sensor --------------------
constexpr uint8_t PIN_BTN_START = 16;
constexpr uint8_t PIN_BTN_STOP = 17;
constexpr uint8_t PIN_BTN_HOME = 18;
constexpr bool BTN_ACTIVE_LOW = true;
constexpr uint32_t BTN_DEBOUNCE_MS = 30;

constexpr uint8_t PIN_SEN_DET = 35;
constexpr bool SENSOR_ACTIVE_LOW = true;
constexpr uint32_t SENSOR_DEBOUNCE_MS = 10;  // covers relay output contact bounce
constexpr uint32_t SENSOR_TRIGGER_COUNT = 2;  // detections while running before the machine pauses

// -------------------- Machine sequence --------------------
// Stroke lengths in steps (3 inch on S_M1/S_M3, 1 inch on S_M2)
constexpr int32_t S_M1_STROKE_STEPS = 5000;
constexpr int32_t S_M2_STROKE_STEPS = 2500;
constexpr int32_t S_M3_STROKE_STEPS = 5000;

constexpr uint32_t DC1_300_RUN_MS = 100; //miliseconds
constexpr uint8_t DC1_300_SPEED = 255; // 0-255, 0 = stop, 255 = full speed
constexpr uint32_t DC2_300_RUN_MS = 353;
constexpr uint8_t DC2_300_SPEED = 255;
constexpr uint32_t DC_3000_RUN_MS = 210;
constexpr uint8_t DC_3000_SPEED = 255;

constexpr uint32_t CYCLE_GAP_MS = 1000;  // pause between two cycles
