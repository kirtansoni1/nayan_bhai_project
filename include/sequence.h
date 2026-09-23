#pragma once

/**
 * Runs one full machine cycle as per the run flow chart.
 * @return false if the cycle was abandoned (HOME pressed while paused).
 */
bool sequence_run_cycle();

/**
 * Turns off DC motors and solenoid, then returns all steppers to 0.
 * @return false if homing was interrupted.
 */
bool sequence_home();
