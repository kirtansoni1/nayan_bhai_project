#pragma once

/**
 * Configures the GREEN and RED LED pins and starts the LED task.
 */
void indicator_init();

/**
 * Sets the normal LED state. While a fault is active, RED shows its blink code instead.
 * Safe to call from any task.
 */
void indicator_set(bool green, bool red);
