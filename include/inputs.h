#pragma once

#include <Arduino.h>

enum class InputId : uint8_t {
  START = 0,
  STOP = 1,
  HOME = 2,
  SENSOR = 3,
};

constexpr uint8_t INPUT_COUNT = 4;

/**
 * Called from the input task on every debounced change.
 * @param active true = pressed / object detected
 */
using InputCallback = void (*)(InputId id, bool active);

/**
 * Configures buttons and sensor with edge interrupts and starts the debounce task.
 */
void inputs_init(InputCallback callback);

/**
 * Current debounced state.
 */
bool input_is_active(InputId id);

const char *input_name(InputId id);
