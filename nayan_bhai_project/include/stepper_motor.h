#pragma once

#include <Arduino.h>
#include "defines.h"

/**
 * Initializes stepper drivers and starts internal service task.
 */
void stepper_init();

/**
 * Sets shared speed profile for all three steppers.
 * @param speed steps/second
 * @param acceleration steps/second^2
 * @param deceleration steps/second^2 (mapped to acceleration because AccelStepper uses one accel profile)
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
