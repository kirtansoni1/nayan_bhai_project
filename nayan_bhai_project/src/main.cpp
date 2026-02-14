#include <Arduino.h>

#include "button_matrix.h"
#include "dc_motor.h"
#include "main.h"
#include "stepper_motor.h"

namespace {
void on_matrix_event(MatrixEvent event) {
  switch (event) {
    case MatrixEvent::S_M1_CW:
      stepper_run_infinite(1, Direction::CW);
      break;
    case MatrixEvent::S_M1_CCW:
      stepper_run_infinite(1, Direction::CCW);
      break;
    case MatrixEvent::S_M1_STOP:
      stepper_stop(1);
      break;
    case MatrixEvent::S_M2_CW:
      stepper_run_infinite(2, Direction::CW);
      break;
    case MatrixEvent::S_M2_CCW:
      stepper_run_infinite(2, Direction::CCW);
      break;
    case MatrixEvent::S_M2_STOP:
      stepper_stop(2);
      break;
    case MatrixEvent::S_M3_CW:
      stepper_run_infinite(3, Direction::CW);
      break;
    case MatrixEvent::S_M3_CCW:
      stepper_run_infinite(3, Direction::CCW);
      break;
    case MatrixEvent::S_M3_STOP:
      stepper_stop(3);
      break;
    default:
      break;
  }
}
}  // namespace

void solenoid_state(SolenoidState state) {
  digitalWrite(static_cast<uint8_t>(PIN_SOLENOID_RLY), state == SolenoidState::ON ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);

  pinMode(static_cast<uint8_t>(PIN_SOLENOID_RLY), OUTPUT);
  solenoid_state(SolenoidState::OFF);

  stepper_init();
  steppr_set_config(1200.0f, 800.0f, 800.0f);

  dc_motor_init();
  dc_stop_all();

  button_matrix_init(on_matrix_event);

  Serial.println("System initialized.");
}

void loop() {
  // All controls run in FreeRTOS tasks. Keep loop non-blocking.
  vTaskDelay(pdMS_TO_TICKS(10));
}