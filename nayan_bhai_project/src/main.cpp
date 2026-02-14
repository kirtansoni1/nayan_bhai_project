#include <Arduino.h>

#include "button_matrix.h"
#include "dc_motor.h"
#include "main.h"
#include "stepper_motor.h"

namespace {
constexpr int32_t JOG_STEPS_PER_KEY = 200;

void on_matrix_event(MatrixEvent event) {
  switch (event) {
    case MatrixEvent::S_M1_CW:
      stepper_run_steps(1, JOG_STEPS_PER_KEY, Direction::CW);
      break;
    case MatrixEvent::S_M1_CCW:
      stepper_run_steps(1, JOG_STEPS_PER_KEY, Direction::CCW);
      break;
    case MatrixEvent::S_M2_CW:
      stepper_run_steps(2, JOG_STEPS_PER_KEY, Direction::CW);
      break;
    case MatrixEvent::S_M2_CCW:
      stepper_run_steps(2, JOG_STEPS_PER_KEY, Direction::CCW);
      break;
    case MatrixEvent::S_M3_CW:
      stepper_run_steps(3, JOG_STEPS_PER_KEY, Direction::CW);
      break;
    case MatrixEvent::S_M3_CCW:
      stepper_run_steps(3, JOG_STEPS_PER_KEY, Direction::CCW);
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