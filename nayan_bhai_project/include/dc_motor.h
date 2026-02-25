#pragma once

#include <Arduino.h>
#include "defines.h"

enum class DcMotorId : uint8_t {
	M3000 = 0,
	M1_300 = 1,
	M2_300 = 2,
};

struct DcTimedMove {
	DcMotorId motor;
	uint32_t time_ms;
	uint8_t speed;
	Direction direction;
};

/**
 * Initializes both DC motor groups.
 */
void dc_motor_init();

/**
 * Services timed DC commands. Call this frequently from loop().
 */
void dc_service();

/**
 * 3000 RPM group control (non-blocking).
 */
void dc_3000_run(uint8_t speed, Direction direction);
void dc_3000_run_ms(uint32_t time_ms, uint8_t speed, Direction direction);
void dc_3000_run_ms_blocking(uint32_t time_ms, uint8_t speed, Direction direction);
void dc_3000_stop();

/**
 * 300 RPM group control (non-blocking).
 */
void dc1_300_run(uint8_t speed, Direction direction);
void dc1_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction);
void dc1_300_run_ms_blocking(uint32_t time_ms, uint8_t speed, Direction direction);
void dc1_300_stop();

void dc2_300_run(uint8_t speed, Direction direction);
void dc2_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction);
void dc2_300_run_ms_blocking(uint32_t time_ms, uint8_t speed, Direction direction);
void dc2_300_stop();

/**
 * Starts multiple timed DC runs together and blocks until all are complete.
 */
void dc_run_ms_batch_blocking(const DcTimedMove *moves, uint8_t move_count);

/**
 * Stops both DC motor groups.
 */
void dc_stop_all();
