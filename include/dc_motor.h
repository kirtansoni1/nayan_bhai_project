#pragma once

#include <Arduino.h>

#include "config.h"

enum class DcMotorId : uint8_t {
  DC1_300 = 0,
  DC2_300 = 1,
  DC_3000 = 2,
};

constexpr uint8_t DC_MOTOR_COUNT = 3;

struct DcTimedMove {
  DcMotorId motor;
  uint32_t time_ms;
  uint8_t speed;  // 0..DC_PWM_MAX
  Direction direction;
};

/**
 * Sets up PWM on all DC drivers and leaves them stopped and disabled.
 * Requires io_expander_init().
 */
void dc_motor_init();

/**
 * Runs a motor until dc_stop() (non-blocking).
 */
void dc_run(DcMotorId motor, uint8_t speed, Direction direction);

/**
 * Stops a motor and disables its driver (coast).
 */
void dc_stop(DcMotorId motor);

void dc_stop_all();

/**
 * Runs one motor for time_ms. Pauses with the machine and resumes the remaining time.
 * @return false if the machine aborted the move.
 */
bool dc_run_ms_blocking(DcMotorId motor, uint32_t time_ms, uint8_t speed, Direction direction);

/**
 * Starts several timed runs together and blocks until all finish. One entry per motor.
 * @return false if the machine aborted the moves.
 */
bool dc_run_ms_batch_blocking(const DcTimedMove *moves, uint8_t move_count);
