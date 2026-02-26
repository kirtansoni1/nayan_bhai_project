#pragma once

#include "defines.h"

extern bool start_button_pressed;
extern volatile bool g_paused;

// Onboard RGB LED helpers
void rgb_led_init();
void set_rgb_led(uint8_t r, uint8_t g, uint8_t b);