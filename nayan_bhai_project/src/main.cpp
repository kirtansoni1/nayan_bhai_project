#include <Arduino.h>

#include "button_matrix.h"
#include "dc_motor.h"
#include "main.h"
#include "stepper_motor.h"

namespace {
const StepperMove Task7[] = {
  {3, 5000, Direction::CW},
  {2, 2500, Direction::CCW},
};

void on_matrix_event(MatrixEvent event) {
  switch (event) {
    case MatrixEvent::S_M1_CW:
      Serial.println("Button0 Pressed: Stepper 1 CW");
      stepper_run_infinite(1, Direction::CW);
      break;
    case MatrixEvent::S_M1_CCW:
      Serial.println("Button1 Pressed: Stepper 1 CCW");
      stepper_run_infinite(1, Direction::CCW);
      break;
    case MatrixEvent::S_M1_STOP:
      Serial.println("Button2 Pressed: Stepper 1 STOP");
      stepper_stop(1);
      break;
    case MatrixEvent::S_M2_CW:
      Serial.println("Button3 Pressed: Stepper 2 CW");
      stepper_run_infinite(2, Direction::CW);
      break;
    case MatrixEvent::S_M2_CCW:
      Serial.println("Button4 Pressed: Stepper 2 CCW");
      stepper_run_infinite(2, Direction::CCW);
      break;
    case MatrixEvent::S_M2_STOP:
      Serial.println("Button5 Pressed: Stepper 2 STOP");
      stepper_stop(2);
      break;
    case MatrixEvent::S_M3_CW:
      Serial.println("Button6 Pressed: Stepper 3 CW");
      stepper_run_infinite(3, Direction::CW);
      break;
    case MatrixEvent::S_M3_CCW:
      Serial.println("Button7 Pressed: Stepper 3 CCW");
      stepper_run_infinite(3, Direction::CCW);
      break;
    case MatrixEvent::S_M3_STOP:
      Serial.println("Button8 Pressed: Stepper 3 STOP");
      stepper_stop(3);
      break;
    default:
      break;
  }
}
}  // namespace

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

  button_matrix_init(on_matrix_event);

  Serial.println("System initialized - sequential loop script mode");
}

void loop() {
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