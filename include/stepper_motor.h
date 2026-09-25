#pragma once

#include <Arduino.h>

#include "config.h"

// Motors are numbered 1..STEPPER_COUNT (S_M1..S_M3). CW = positive position.

struct StepperMove {
  uint8_t motor_number;
  int32_t steps;  // > 0
  Direction direction;
};

/**
 * Sets up step/dir pins. Position of every motor = 0.
 * Drivers are enabled by io_expander_init() at boot and never toggled.
 */
void stepper_init();

/**
 * Speed profile shared by all motors.
 * @param speed steps/s
 * @param acceleration steps/s^2, or -1 to run at constant speed with no ramp
 */
void stepper_set_config(float speed, float acceleration);

/**
 * Moves one motor and blocks until done. Pauses and resumes with the machine.
 * @return false if the machine aborted the move.
 */
bool stepper_run_steps_blocking(uint8_t motor_number, int32_t steps, Direction direction);

/**
 * Moves several motors together and blocks until all are done. One entry per motor.
 * @return false if the machine aborted the moves.
 */
bool stepper_run_steps_batch_blocking(const StepperMove *moves, uint8_t move_count);

/**
 * Moves all motors to position 0 together.
 * @return false if the machine aborted homing.
 */
bool stepper_home_all_blocking();

/**
 * Tracked position in steps, 0 = home.
 */
int32_t stepper_get_position(uint8_t motor_number);
