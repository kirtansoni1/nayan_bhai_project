#pragma once

/**
 * Runs one full machine cycle as per the run flow chart.
 * Opens the sensor window after task 9.
 * @return false if the cycle was aborted by a fault.
 */
bool sequence_run_cycle();

/**
 * HOME button action. Turns off DC motors and solenoid, moves S_M3 its fixed jog,
 * then takes wherever it ended as the new S_M3 zero.
 * @return false if the jog was stopped early.
 */
bool sequence_jog();
