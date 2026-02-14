#include "stepper_motor.h"

#include <AccelStepper.h>

namespace {
AccelStepper steppers[MOTOR_COUNT] = {
    AccelStepper(AccelStepper::DRIVER, static_cast<uint8_t>(PIN_S_M1_STEP), static_cast<uint8_t>(PIN_S_M1_DIR)),
    AccelStepper(AccelStepper::DRIVER, static_cast<uint8_t>(PIN_S_M2_STEP), static_cast<uint8_t>(PIN_S_M2_DIR)),
    AccelStepper(AccelStepper::DRIVER, static_cast<uint8_t>(PIN_S_M3_STEP), static_cast<uint8_t>(PIN_S_M3_DIR)),
};

struct StepperRuntime {
  bool timedRunActive;
  uint32_t timedRunEndMs;
};

StepperRuntime runtime[MOTOR_COUNT] = {};

float g_maxSpeed = STEPPER_DEFAULT_MAX_SPEED;
float g_acceleration = STEPPER_DEFAULT_ACCEL;
float g_deceleration = STEPPER_DEFAULT_DECEL;

portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t stepperTaskHandle = nullptr;

bool is_valid_motor(uint8_t motor_number) {
  return motor_number >= 1 && motor_number <= MOTOR_COUNT;
}

uint8_t idx_from_motor(uint8_t motor_number) {
  return motor_number - 1;
}

void stepper_service_task(void *parameter) {
  (void)parameter;

  for (;;) {
    const uint32_t now = millis();

    taskENTER_CRITICAL(&stepperMux);
    for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
      if (runtime[index].timedRunActive && static_cast<int32_t>(now - runtime[index].timedRunEndMs) >= 0) {
        steppers[index].stop();
        runtime[index].timedRunActive = false;
      }
      steppers[index].run();
    }
    taskEXIT_CRITICAL(&stepperMux);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
}  // namespace

void stepper_init() {
  pinMode(static_cast<uint8_t>(PIN_S_M_EN), OUTPUT);
  stepper_enable(true);

  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    steppers[index].setMaxSpeed(g_maxSpeed);
    steppers[index].setAcceleration(g_acceleration);
    steppers[index].setCurrentPosition(0);
  }

  if (stepperTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(stepper_service_task, "stepper_task", 4096, nullptr, 3, &stepperTaskHandle, 1);
  }
}

void steppr_set_config(float speed, float acceleration, float deceleration) {
  if (speed <= 0.0f) {
    speed = STEPPER_DEFAULT_MAX_SPEED;
  }
  if (acceleration <= 0.0f) {
    acceleration = STEPPER_DEFAULT_ACCEL;
  }
  if (deceleration <= 0.0f) {
    deceleration = STEPPER_DEFAULT_DECEL;
  }

  g_maxSpeed = speed;
  g_acceleration = acceleration;
  g_deceleration = deceleration;

  taskENTER_CRITICAL(&stepperMux);
  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    steppers[index].setMaxSpeed(g_maxSpeed);
    steppers[index].setAcceleration(g_acceleration);
  }
  taskEXIT_CRITICAL(&stepperMux);
}

void stepper_set_config(float speed, float acceleration, float deceleration) {
  steppr_set_config(speed, acceleration, deceleration);
}

void stepper_run_ms(uint8_t motor_number, uint32_t time_ms, Direction direction) {
  if (!is_valid_motor(motor_number) || time_ms == 0) {
    return;
  }

  const uint8_t index = idx_from_motor(motor_number);
  const int32_t farTargetOffset = (direction == Direction::CW) ? 2000000000L : -2000000000L;

  taskENTER_CRITICAL(&stepperMux);
  steppers[index].move(farTargetOffset);
  runtime[index].timedRunActive = true;
  runtime[index].timedRunEndMs = millis() + time_ms;
  taskEXIT_CRITICAL(&stepperMux);
}

void stepper_run_steps(uint8_t motor_number, int32_t steps, Direction direction) {
  if (!is_valid_motor(motor_number) || steps <= 0) {
    return;
  }

  const uint8_t index = idx_from_motor(motor_number);
  const int32_t signedSteps = (direction == Direction::CW) ? steps : -steps;

  taskENTER_CRITICAL(&stepperMux);
  runtime[index].timedRunActive = false;
  steppers[index].move(signedSteps);
  taskEXIT_CRITICAL(&stepperMux);
}

void stepper_stop(uint8_t motor_number) {
  if (!is_valid_motor(motor_number)) {
    return;
  }

  const uint8_t index = idx_from_motor(motor_number);
  taskENTER_CRITICAL(&stepperMux);
  runtime[index].timedRunActive = false;
  steppers[index].stop();
  taskEXIT_CRITICAL(&stepperMux);
}

void stepper_all_stop() {
  taskENTER_CRITICAL(&stepperMux);
  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    runtime[index].timedRunActive = false;
    steppers[index].stop();
  }
  taskEXIT_CRITICAL(&stepperMux);
}

int32_t stepper_get_position(uint8_t motor_number) {
  if (!is_valid_motor(motor_number)) {
    return 0;
  }

  const uint8_t index = idx_from_motor(motor_number);

  taskENTER_CRITICAL(&stepperMux);
  const int32_t position = steppers[index].currentPosition();
  taskEXIT_CRITICAL(&stepperMux);

  return position;
}

void stepper_enable(bool enabled) {
  // DM542 EN input is commonly active LOW. Set LOW to enable drivers, HIGH to disable.
  digitalWrite(static_cast<uint8_t>(PIN_S_M_EN), enabled ? LOW : HIGH);
}
