#pragma once

/**
 * Switches the 12 V solenoid relay. Requires io_expander_init().
 */
void solenoid_set(bool on);

/**
 * Last commanded state.
 */
bool solenoid_is_on();
