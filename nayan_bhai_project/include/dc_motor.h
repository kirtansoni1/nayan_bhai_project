#pragma once

#include <Arduino.h>
#include "defines.h"

/**
 * Initializes both DC motor groups and starts service task.
 */
void dc_motor_init();

/**
 * 3000 RPM group control (non-blocking).
 */
void dc_3000_run(uint8_t speed, Direction direction);
void dc_3000_run_ms(uint32_t time_ms, uint8_t speed, Direction direction);
void dc_3000_stop();

/**
 * 300 RPM group control (non-blocking).
 */
void dc_300_run(uint8_t speed, Direction direction);
void dc_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction);
void dc_300_stop();

/**
 * Stops both DC motor groups.
 */
void dc_stop_all();
