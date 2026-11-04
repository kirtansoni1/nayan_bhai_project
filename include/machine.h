#pragma once

#include <Arduino.h>

#include "inputs.h"

enum class MachineState : uint8_t {
  STOPPED,  // idle, next START begins a fresh cycle
  RUNNING,  // cycle in progress
  PAUSED,   // cycle frozen mid-way, START resumes it
  JOGGING,  // HOME pressed, S_M3 moving its fixed jog
};

// What the main task is currently executing
enum class Activity : uint8_t {
  NONE,
  CYCLE,
  JOG,
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
 * Main task reports the end of a cycle. Closes the sensor window and
 * stops the machine if the sensor limit was reached.
 * @param completed false if the cycle was aborted
 */
void machine_cycle_finished(bool completed);

/**
 * Main task reports the end of a jog, done or stopped.
 */
void machine_jog_finished();

/**
 * Forces STOPPED after a fatal fault. Called by the main task.
 */
void machine_fault_stop();
