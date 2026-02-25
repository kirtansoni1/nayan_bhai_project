#include "stepper_motor.h"

#include <AccelStepper.h>

namespace {
AccelStepper steppers[STEPPER_MOTOR_COUNT] = {
    AccelStepper(AccelStepper::DRIVER, static_cast<uint8_t>(PIN_S_M1_STEP), static_cast<uint8_t>(PIN_S_M1_DIR)),
    AccelStepper(AccelStepper::DRIVER, static_cast<uint8_t>(PIN_S_M2_STEP), static_cast<uint8_t>(PIN_S_M2_DIR)),
    AccelStepper(AccelStepper::DRIVER, static_cast<uint8_t>(PIN_S_M3_STEP), static_cast<uint8_t>(PIN_S_M3_DIR)),
};

struct StepperRuntime {
  bool timedRunActive;
  uint32_t timedRunEndMs;
  bool infiniteRunActive;
  bool stepRunActive;
  int32_t stepRunTarget;
  int8_t stepRunDirection;
};

StepperRuntime runtime[STEPPER_MOTOR_COUNT] = {};

float g_maxSpeed = STEPPER_DEFAULT_MAX_SPEED;
float g_acceleration = STEPPER_DEFAULT_ACCEL;
float g_deceleration = STEPPER_DEFAULT_DECEL;
bool g_noRampMode = false;

bool is_valid_motor(uint8_t motor_number) {
  return motor_number >= 1 && motor_number <= STEPPER_MOTOR_COUNT;
}

uint8_t idx_from_motor(uint8_t motor_number) {
  return motor_number - 1;
}

bool is_motor_motion_complete(uint8_t index) {
  const bool stepRunActive = runtime[index].stepRunActive;
  const bool infiniteRunActive = runtime[index].infiniteRunActive;
  const bool timedRunActive = runtime[index].timedRunActive;
  const int32_t remainingSteps = steppers[index].distanceToGo();
  return !stepRunActive && !infiniteRunActive && !timedRunActive && (remainingSteps == 0);
}
}  // namespace

void stepper_init() {
  pinMode(static_cast<uint8_t>(PIN_S_M_EN), OUTPUT);
  stepper_enable(true);

  for (uint8_t index = 0; index < STEPPER_MOTOR_COUNT; ++index) {
    steppers[index].setMaxSpeed(g_maxSpeed);
    steppers[index].setAcceleration(g_acceleration);
    steppers[index].setCurrentPosition(0);
  }
}

void stepper_service() {
  const uint32_t now = millis();

  for (uint8_t index = 0; index < STEPPER_MOTOR_COUNT; ++index) {
    if (runtime[index].timedRunActive && static_cast<int32_t>(now - runtime[index].timedRunEndMs) >= 0) {
      runtime[index].timedRunActive = false;
      if (g_noRampMode) {
        steppers[index].setSpeed(0.0f);
      } else {
        steppers[index].stop();
      }
    }

    if (runtime[index].stepRunActive) {
      const int32_t currentPosition = steppers[index].currentPosition();
      const bool reachedTarget = (runtime[index].stepRunDirection > 0)
                                   ? (currentPosition >= runtime[index].stepRunTarget)
                                   : (currentPosition <= runtime[index].stepRunTarget);
      if (reachedTarget) {
        steppers[index].setCurrentPosition(runtime[index].stepRunTarget);
        steppers[index].setSpeed(0.0f);
        runtime[index].stepRunActive = false;
      } else {
        steppers[index].runSpeed();
      }
    } else if (runtime[index].infiniteRunActive || (g_noRampMode && runtime[index].timedRunActive)) {
      steppers[index].runSpeed();
    } else {
      steppers[index].run();
    }
  }
}

void steppr_set_config(float speed, float acceleration, float deceleration) {
  if (speed <= 0.0f) {
    speed = STEPPER_DEFAULT_MAX_SPEED;
  }

  const bool noRampRequested = (acceleration == -1.0f) || (deceleration == -1.0f);
  if (noRampRequested) {
    acceleration = STEPPER_DEFAULT_ACCEL;
    deceleration = STEPPER_DEFAULT_DECEL;
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
  g_noRampMode = noRampRequested;

  for (uint8_t index = 0; index < STEPPER_MOTOR_COUNT; ++index) {
    steppers[index].setMaxSpeed(g_maxSpeed);
    steppers[index].setAcceleration(g_acceleration);
  }
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
  const float signedSpeed = (direction == Direction::CW) ? g_maxSpeed : -g_maxSpeed;

  runtime[index].stepRunActive = false;
  runtime[index].infiniteRunActive = false;
  if (g_noRampMode) {
    steppers[index].setSpeed(signedSpeed);
  } else {
    steppers[index].move(farTargetOffset);
  }
  runtime[index].timedRunActive = true;
  runtime[index].timedRunEndMs = millis() + time_ms;
}

void stepper_run_steps(uint8_t motor_number, int32_t steps, Direction direction) {
  if (!is_valid_motor(motor_number) || steps <= 0) {
    return;
  }

  const uint8_t index = idx_from_motor(motor_number);
  const int32_t signedSteps = (direction == Direction::CW) ? steps : -steps;

  runtime[index].infiniteRunActive = false;
  runtime[index].timedRunActive = false;
  if (g_noRampMode) {
    runtime[index].stepRunActive = true;
    runtime[index].stepRunDirection = (signedSteps > 0) ? 1 : -1;
    runtime[index].stepRunTarget = steppers[index].currentPosition() + signedSteps;
    steppers[index].setSpeed((signedSteps > 0) ? g_maxSpeed : -g_maxSpeed);
  } else {
    runtime[index].stepRunActive = false;
    steppers[index].move(signedSteps);
  }
}

void stepper_run_steps_blocking(uint8_t motor_number, int32_t steps, Direction direction) {
  if (!is_valid_motor(motor_number) || steps <= 0) {
    return;
  }

  stepper_run_steps(motor_number, steps, direction);

  const uint8_t index = idx_from_motor(motor_number);
  for (;;) {
    stepper_service();

    if (is_motor_motion_complete(index)) {
      break;
    }

    delay(0);
  }
}

void stepper_run_steps_batch_blocking(const StepperMove *moves, uint8_t move_count) {
  if (moves == nullptr || move_count == 0) {
    return;
  }

  bool hasValidMove = false;
  for (uint8_t moveIndex = 0; moveIndex < move_count; ++moveIndex) {
    if (is_valid_motor(moves[moveIndex].motor_number) && moves[moveIndex].steps > 0) {
      hasValidMove = true;
      stepper_run_steps(moves[moveIndex].motor_number, moves[moveIndex].steps, moves[moveIndex].direction);
    }
  }

  if (!hasValidMove) {
    return;
  }

  for (;;) {
    bool allComplete = true;

    stepper_service();

    for (uint8_t moveIndex = 0; moveIndex < move_count; ++moveIndex) {
      const StepperMove &move = moves[moveIndex];
      if (!is_valid_motor(move.motor_number) || move.steps <= 0) {
        continue;
      }

      const uint8_t motorIndex = idx_from_motor(move.motor_number);
      if (!is_motor_motion_complete(motorIndex)) {
        allComplete = false;
        break;
      }
    }

    if (allComplete) {
      break;
    }

    delay(0);
  }
}

void stepper_run_infinite(uint8_t motor_number, Direction direction) {
  if (!is_valid_motor(motor_number)) {
    return;
  }

  const uint8_t index = idx_from_motor(motor_number);
  const float signedSpeed = (direction == Direction::CW) ? g_maxSpeed : -g_maxSpeed;

  runtime[index].stepRunActive = false;
  runtime[index].timedRunActive = false;
  runtime[index].infiniteRunActive = true;
  steppers[index].setSpeed(signedSpeed);
}

void stepper_stop(uint8_t motor_number) {
  if (!is_valid_motor(motor_number)) {
    return;
  }

  const uint8_t index = idx_from_motor(motor_number);
  runtime[index].stepRunActive = false;
  runtime[index].infiniteRunActive = false;
  runtime[index].timedRunActive = false;
  if (g_noRampMode) {
    steppers[index].setSpeed(0.0f);
  } else {
    steppers[index].stop();
  }
}

void stepper_all_stop() {
  for (uint8_t index = 0; index < STEPPER_MOTOR_COUNT; ++index) {
    runtime[index].stepRunActive = false;
    runtime[index].infiniteRunActive = false;
    runtime[index].timedRunActive = false;
    if (g_noRampMode) {
      steppers[index].setSpeed(0.0f);
    } else {
      steppers[index].stop();
    }
  }
}

int32_t stepper_get_position(uint8_t motor_number) {
  if (!is_valid_motor(motor_number)) {
    return 0;
  }

  const uint8_t index = idx_from_motor(motor_number);
  const int32_t position = steppers[index].currentPosition();

  return position;
}

void stepper_enable(bool enabled) {
  // DM542 EN input is commonly active LOW. Set LOW to enable drivers, HIGH to disable.
  digitalWrite(static_cast<uint8_t>(PIN_S_M_EN), enabled ? LOW : HIGH);
}
