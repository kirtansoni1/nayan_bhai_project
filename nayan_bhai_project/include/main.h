#pragma once

#include "defines.h"

/**
 * Controls the solenoid relay output.
 */
void solenoid_state(SolenoidState state);

/**
 * Runs one full blocking motion sequence.
 * Edit this implementation to define custom process steps.
 */
void run_sequence_blocking();
