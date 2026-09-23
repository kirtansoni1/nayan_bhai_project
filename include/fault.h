#pragma once

#include <Arduino.h>

// Value = number of RED blinks. Higher value = more severe, shown first.
enum class Fault : uint8_t {
  SENSOR_LIMIT = 1,      // sensor reached SENSOR_TRIGGER_COUNT detections
  HOME_REQUIRED = 2,     // homing was interrupted, axes not at 0
  UNEXPECTED_RESET = 3,  // brownout / crash / watchdog, tracked position lost
  IO_EXPANDER = 4,       // TCA9535 not responding, drivers cannot be controlled
};

/**
 * Raises a fault. Prints once when it becomes active. Safe from any task.
 */
void fault_set(Fault fault);

/**
 * Clears a fault. Safe from any task.
 */
void fault_clear(Fault fault);

bool fault_active(Fault fault);

/**
 * Most severe active fault as its blink count, 0 when none.
 */
uint8_t fault_code();
