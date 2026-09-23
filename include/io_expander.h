#pragma once

#include <Arduino.h>

/**
 * Starts I2C and configures the TCA9535 outputs in their safe (off) state.
 * Output levels are written before the pins switch to output, so nothing glitches on.
 * @return false if the expander does not answer.
 */
bool io_expander_init();

/**
 * Sets one expander output pin. Only call from the main task.
 * Retries on I2C error, then raises Fault::IO_EXPANDER.
 * @return false on I2C error.
 */
bool io_expander_write(uint8_t pin, bool level);
