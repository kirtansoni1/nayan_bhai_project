#pragma once

#include <Arduino.h>

/**
 * Starts I2C and configures the TCA9535 outputs: DC drivers and solenoid off,
 * stepper drivers enabled (fixed from here on).
 * Output levels are written before the pins switch to output, so nothing glitches on.
 * @return false if the expander does not answer.
 */
bool io_expander_init();

/**
 * Sets one expander output pin and reads the pins back to prove it.
 * Stepper enable pins are fixed at boot and cannot be written.
 * On mismatch the expander setup is restored, then Fault::IO_EXPANDER if it still fails.
 * Only call from the main task.
 * @return false on failure.
 */
bool io_expander_write(uint8_t pin, bool level);

/**
 * Reads all output pins back and restores them if they drifted (noise, expander reset).
 * Only call from the main task.
 * @return false on failure.
 */
bool io_expander_check();
