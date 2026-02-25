#pragma once

#include <Arduino.h>
#include "defines.h"

struct StepperMove {
	uint8_t motor_number;
	int32_t steps;
	Direction direction;
};

/**
 * Initializes stepper drivers.
 */
void stepper_init();

/**
 * Services all steppers. Call this frequently from loop().
 */
void stepper_service();

/**
 * Sets shared speed profile for all three steppers.
 * @param speed steps/second
 * @param acceleration steps/second^2, or -1 to disable ramping (instant speed changes)
 * @param deceleration steps/second^2, or -1 to disable ramping (instant speed changes)
 */
void steppr_set_config(float speed, float acceleration, float deceleration);

/**
 * Alias for typo-safe API naming.
 */
void stepper_set_config(float speed, float acceleration, float deceleration);

/**
 * Runs a motor for a given time in milliseconds (non-blocking).
 */
void stepper_run_ms(uint8_t motor_number, uint32_t time_ms, Direction direction);

/**
 * Runs a motor for a given number of steps (non-blocking).
 */
void stepper_run_steps(uint8_t motor_number, int32_t steps, Direction direction);

/**
 * Runs a motor for a given number of steps and blocks until target is reached.
 */
void stepper_run_steps_blocking(uint8_t motor_number, int32_t steps, Direction direction);

/**
 * Starts multiple step runs together and blocks until all of them are complete.
 */
void stepper_run_steps_batch_blocking(const StepperMove *moves, uint8_t move_count);

/**
 * Runs a motor continuously until explicitly stopped (non-blocking).
 */
void stepper_run_infinite(uint8_t motor_number, Direction direction);

/**
 * Immediately stops one stepper.
 */
void stepper_stop(uint8_t motor_number);

/**
 * Immediately stops all steppers.
 */
void stepper_all_stop();

/**
 * Returns tracked current position in steps.
 */
int32_t stepper_get_position(uint8_t motor_number);

/**
 * Enables or disables all stepper drivers through shared EN pin.
 */
void stepper_enable(bool enabled);
