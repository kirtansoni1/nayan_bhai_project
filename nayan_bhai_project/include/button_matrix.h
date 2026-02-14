#pragma once

#include <Arduino.h>

enum class MatrixEvent : uint8_t {
  NONE = 0,
  S_M1_CW,
  S_M1_CCW,
  S_M2_CW,
  S_M2_CCW,
  S_M3_CW,
  S_M3_CCW,
};

using MatrixEventCallback = void (*)(MatrixEvent event);

/**
 * Initializes matrix keyboard and ISR-assisted scanning task.
 */
void button_matrix_init(MatrixEventCallback callback);
