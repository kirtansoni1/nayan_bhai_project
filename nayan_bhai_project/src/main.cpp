#include <Arduino.h>

#include "button_matrix.h"
#include "dc_motor.h"
#include "main.h"
#include "stepper_motor.h"

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
    }
  }

  if (event.button == ButtonId::BTNB) {
    if (event.state == ButtonState::PRESSED) {
      g_paused = true;
      // Instantly stop all actuators
      stepper_all_stop();
      dc_stop_all();
      Serial.println("[BTN] PAUSE - press A to resume");
    }
  }
}

void solenoid_state(SolenoidState state) {
  digitalWrite(static_cast<uint8_t>(PIN_SOLENOID_RLY), state == SolenoidState::ON ? LOW : HIGH);
}

void setup() {
  Serial.begin(115200);

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
  dc1_300_run_ms_blocking(100, 255, Direction::CW); // Task1: Run 300 RPM DC motor1 clockwise
  stepper_run_steps_blocking(1, 5000, Direction::CCW); // Task2: Run stepper 1 counterclockwise for 3 inch (set steps of the motor)
  stepper_run_steps_blocking(2, 2500, Direction::CW); // Task3: Run stepper 2 clockwise for 1 inch (set steps of the motor)
  solenoid_state(SolenoidState::ON); // Task4: Turn on the solenoid
  dc2_300_run_ms_blocking(353, 255, Direction::CW); // Task5: Run 300 RPM DC motor2 clockwise
  stepper_run_steps_blocking(1, 5000, Direction::CW); // Task6: Run stepper 1 clockwise for 3 inch (set steps of the motor)
  stepper_run_steps_batch_blocking(Task7, static_cast<uint8_t>(sizeof(Task7) / sizeof(Task7[0]))); // Task7: Run Stepper 3 clockwise for 3 inch and Stepper 2 counterclockwise for 1 inch at the same time (set steps of the motor)
  dc_3000_run_ms_blocking(210, 100, Direction::CW); // Task8: Run 3000 RPM DC motor clockwise
  stepper_run_steps_blocking(3, 5000, Direction::CCW); // Task9: Run stepper 3 counterclockwise for 3 inch (set steps of the motor)
  solenoid_state(SolenoidState::OFF); // Task10.1: Turn off the solenoid
  dc2_300_run_ms_blocking(353, 255, Direction::CCW); // Task10.2: Run 300 RPM DC motor2 counterclockwise
  // Small gap before repeating the sequence
  delay(1000);
  }
  delay(100);
}