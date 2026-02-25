#pragma once

#include <Arduino.h>

// Physical 4x4 keypad layout:
//  BTN1  BTN2  BTN3  BTNA   <- Line 1 (Row 0)
//  BTN4  BTN5  BTN6  BTNB   <- Line 2 (Row 1)
//  BTN7  BTN8  BTN9  BTNC   <- Line 3 (Row 2)
//  BTNSTAR BTN0 BTNHASH BTND <- Line 4 (Row 3)

enum class ButtonId : uint8_t {
  BTN1,
  BTN2,
  BTN3,
  BTNA,
  BTN4,
  BTN5,
  BTN6,
  BTNB,
  BTN7,
  BTN8,
  BTN9,
  BTNC,
  BTNSTAR,
  BTN0,
  BTNHASH,
  BTND,
  UNKNOWN,
};

enum class ButtonState : uint8_t {
  PRESSED,
  RELEASED,
};

struct ButtonEvent {
  ButtonId  button;
  ButtonState state;
};

/**
 * Returns a human-readable name for a button (e.g. "BTN1", "BTNA").
 * Useful for Serial debug prints.
 */
const char* button_name(ButtonId id);

using ButtonEventCallback = void (*)(ButtonEvent event);

/**
 * Initializes the 4x4 keypad and starts a background polling task.
 * Fires callback with ButtonEvent on every press and release.
 */
void button_matrix_init(ButtonEventCallback callback);
