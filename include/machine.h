#pragma once

#include <Arduino.h>

#include "inputs.h"

enum class MachineState : uint8_t {
  STOPPED,  // idle, next START begins a fresh cycle
  RUNNING,  // cycle in progress
  PAUSED,   // cycle frozen mid-way, START resumes it
  HOMING,   // steppers returning to 0
};

// What the main task is currently executing
enum class Activity : uint8_t {
  NONE,
  CYCLE,
  HOMING,
};

/**
 * Creates the state lock and sets STOPPED. Call before inputs_init().
 */
void machine_init();

/**
 * Input handler. Runs in the input task and only changes state, never drives hardware.
 */
void machine_on_input(InputId id, bool active);

MachineState machine_state();

/**
 * Main task tells the gate what it is running, so pause/abort rules apply correctly.
 */
void machine_set_activity(Activity activity);

/**
 * Forces STOPPED after a fatal fault. Called by the main task.
 */
void machine_fault_stop();

/**
 * Main task reports the end of homing.
 * @param completed false if homing was interrupted
 */
void machine_homing_finished(bool completed);
