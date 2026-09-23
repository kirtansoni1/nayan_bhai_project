#pragma once

// Motion gate used by the blocking motor functions.
// Implemented by the machine state logic.

/**
 * True when running motion must halt now (paused, stopped or aborted).
 */
bool run_halted();

/**
 * Blocks while the machine is paused.
 * @return true to continue the current motion, false if it must be abandoned.
 */
bool run_wait_resume();
