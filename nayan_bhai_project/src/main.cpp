#include <Arduino.h>
#include <FastLED.h>

#include "button_matrix.h"
#include "dc_motor.h"
#include "main.h"
#include "stepper_motor.h"

static CRGB g_leds[1];

void rgb_led_init() {
  FastLED.addLeds<WS2812, PIN_RGB_LED, GRB>(g_leds, 1);
  FastLED.setBrightness(RGB_LED_BRIGHTNESS);
}

void set_rgb_led(uint8_t r, uint8_t g, uint8_t b) {
  g_leds[0] = CRGB(r, g, b);
  FastLED.show();
}

// Global state definitions
bool start_button_pressed = false;
volatile bool g_paused = false;

namespace {
const StepperMove Task7[] = {
  {3, 5000, Direction::CW},
  {2, 2500, Direction::CCW},
};
}

void on_button_event(ButtonEvent event) {
  const char* name = button_name(event.button);
  const char* state = (event.state == ButtonState::PRESSED) ? "PRESSED" : "RELEASED";
  Serial.printf("[BTN] %s %s\n", name, state);

  // BTN1 = Stepper 1 CW (hold) / STOP (release)
  if (event.button == ButtonId::BTN1) {
    // if (event.state == ButtonState::PRESSED)  stepper_run_infinite(1, Direction::CW);
    // if (event.state == ButtonState::RELEASED) stepper_stop(1);
  }

  // BTN4 = Stepper 1 CCW (hold) / STOP (release)
  if (event.button == ButtonId::BTN4) {
    // if (event.state == ButtonState::PRESSED)  stepper_run_infinite(1, Direction::CCW);
    // if (event.state == ButtonState::RELEASED) stepper_stop(1);
  }

  // BTN2 = Stepper 2 CW (hold) / STOP (release)
  if (event.button == ButtonId::BTN2) {
    // if (event.state == ButtonState::PRESSED)  stepper_run_infinite(2, Direction::CW);
    // if (event.state == ButtonState::RELEASED) stepper_stop(2);
  }

  // BTN5 = Stepper 2 CCW (hold) / STOP (release)
  if (event.button == ButtonId::BTN5) {
    // if (event.state == ButtonState::PRESSED)  stepper_run_infinite(2, Direction::CCW);
    // if (event.state == ButtonState::RELEASED) stepper_stop(2);
  }

  // BTN3 = Stepper 3 CW (hold) / STOP (release)
  if (event.button == ButtonId::BTN3) {
    // if (event.state == ButtonState::PRESSED)  stepper_run_infinite(3, Direction::CW);
    // if (event.state == ButtonState::RELEASED) stepper_stop(3);
  }

  // BTN6 = Stepper 3 CCW (hold) / STOP (release)
  if (event.button == ButtonId::BTN6) {
    // if (event.state == ButtonState::PRESSED)  stepper_run_infinite(3, Direction::CCW);
    // if (event.state == ButtonState::RELEASED) stepper_stop(3);
  }

  if (event.button == ButtonId::BTNA) {
    if (event.state == ButtonState::PRESSED) {
      g_paused = false;
      start_button_pressed = true;
      set_rgb_led(0, 255, 0); // GREEN = running
    }
  }

  if (event.button == ButtonId::BTNB) {
    if (event.state == ButtonState::PRESSED) {
      g_paused = true;
      // Instantly stop all actuators
      stepper_all_stop();
      dc_stop_all();
      set_rgb_led(255, 0, 0); // RED = paused
      Serial.println("[BTN] PAUSE - press A to resume");
    }
  }
}

void solenoid_state(SolenoidState state) {
  digitalWrite(static_cast<uint8_t>(PIN_SOLENOID_RLY), state == SolenoidState::ON ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);

  rgb_led_init();
  set_rgb_led(255, 255, 255); // WHITE = waiting for start

  pinMode(static_cast<uint8_t>(PIN_SOLENOID_RLY), OUTPUT);
  solenoid_state(SolenoidState::OFF); // Turn OFF solenoid at startup

  // Initialize framework with acceleration
  stepper_init();
  stepper_set_config(12000.0f, 8000.0f, 8000.0f);  // 12000 steps/sec, 8000 accel

  dc_motor_init();
  dc_stop_all();

  button_matrix_init(on_button_event);

  Serial.println("System initialized - sequential loop script mode");
}

void loop() {
  while(start_button_pressed){
  // Task1: Run 300 RPM DC motor1 clockwise
  dc1_300_run_ms_blocking(100, 255, Direction::CW);

  // Task2: Run stepper 1 counterclockwise for 3 inch (set steps of the motor)
  stepper_run_steps_blocking(1, 5000, Direction::CCW);

  // Task3: Run stepper 2 clockwise for 1 inch (set steps of the motor)
  stepper_run_steps_blocking(2, 2500, Direction::CW);

  // Task4: Turn on the solenoid
  solenoid_state(SolenoidState::ON);

  // Task5: Run 300 RPM DC motor2 clockwise
  dc2_300_run_ms_blocking(353, 255, Direction::CW);

  // Task6: Run stepper 1 clockwise for 3 inch (set steps of the motor)
  stepper_run_steps_blocking(1, 5000, Direction::CW);

  // Task7: Run Stepper 3 clockwise for 3 inch and Stepper 2 counterclockwise for 1 inch at the same time (set steps of the motor)
  stepper_run_steps_batch_blocking(Task7, static_cast<uint8_t>(sizeof(Task7) / sizeof(Task7[0])));

  // Task8: Run 3000 RPM DC motor clockwise
  dc_3000_run_ms_blocking(210, 100, Direction::CW);

  // Task9: Run stepper 3 counterclockwise for 3 inch (set steps of the motor)
  stepper_run_steps_blocking(3, 5000, Direction::CCW);

  // Task10: Turn off the solenoid and Run 300 RPM DC motor2 counterclockwise at the same time
  solenoid_state(SolenoidState::OFF); // Task10.1: Turn off the solenoid
  dc2_300_run_ms_blocking(353, 255, Direction::CCW); // Task10.2: Run 300 RPM DC motor2 counterclockwise

  // Small gap before repeating the sequence
  delay(1000);
  }
  delay(100);
}