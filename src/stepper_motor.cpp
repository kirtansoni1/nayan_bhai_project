#include "stepper_motor.h"

#include <AccelStepper.h>

#include "io_expander.h"
#include "run_control.h"

namespace {

struct StepperPins {
  uint8_t step;
  uint8_t dir;
  uint8_t exio_en;
  bool dir_invert;
};

const StepperPins kPins[STEPPER_COUNT] = {
  {PIN_S_M1_STEP, PIN_S_M1_DIR, EXIO_S_M1_EN, S_M1_DIR_INVERT},
  {PIN_S_M2_STEP, PIN_S_M2_DIR, EXIO_S_M2_EN, S_M2_DIR_INVERT},
  {PIN_S_M3_STEP, PIN_S_M3_DIR, EXIO_S_M3_EN, S_M3_DIR_INVERT},
};

// Outputs are enabled in stepper_init(), not in the global constructor
AccelStepper steppers[STEPPER_COUNT] = {
  AccelStepper(AccelStepper::DRIVER, PIN_S_M1_STEP, PIN_S_M1_DIR, 0xff, 0xff, false),
  AccelStepper(AccelStepper::DRIVER, PIN_S_M2_STEP, PIN_S_M2_DIR, 0xff, 0xff, false),
  AccelStepper(AccelStepper::DRIVER, PIN_S_M3_STEP, PIN_S_M3_DIR, 0xff, 0xff, false),
};

float g_max_speed = STEPPER_MAX_SPEED;
float g_accel = STEPPER_ACCEL;
bool g_no_ramp = false;

bool valid(uint8_t motor_number) {
  return motor_number >= 1 && motor_number <= STEPPER_COUNT;
}

// AccelStepper writes DIR in the same instant as STEP HIGH. DM542 needs DIR
// settled first, so set it here and wait before the first pulse.
void prime_dir(const bool active[], const int32_t target[]) {
  bool changed = false;
  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    if (!active[i]) {
      continue;
    }
    const int32_t dist = target[i] - steppers[i].currentPosition();
    if (dist == 0) {
      continue;
    }
    const bool level = (dist > 0) != kPins[i].dir_invert;
    digitalWrite(kPins[i].dir, level ? HIGH : LOW);
    changed = true;
  }
  if (changed) {
    delayMicroseconds(STEPPER_DIR_SETUP_US);
  }
}

void start(uint8_t i, int32_t target) {
  if (g_no_ramp) {
    const bool forward = target > steppers[i].currentPosition();
    steppers[i].setSpeed(forward ? g_max_speed : -g_max_speed);
  } else {
    steppers[i].moveTo(target);
  }
}

// Steps the motor if due. Returns true while it still has to move.
bool service(uint8_t i, int32_t target) {
  if (g_no_ramp) {
    if (steppers[i].currentPosition() == target) {
      return false;
    }
    steppers[i].runSpeed();
    return steppers[i].currentPosition() != target;
  }
  return steppers[i].run();
}

// Brings active motors to rest. Position stays tracked, so the move can resume.
void halt(const bool active[]) {
  const bool instant = g_no_ramp || STEPPER_HALT_DECEL <= 0.0f;

  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    if (!active[i]) {
      continue;
    }
    AccelStepper &s = steppers[i];
    if (instant || s.speed() == 0.0f) {
      s.setCurrentPosition(s.currentPosition());  // zero speed, keep position
      continue;
    }

    // Stop within the halt deceleration, but never past the original target
    s.setAcceleration(STEPPER_HALT_DECEL);
    const float v = s.speed();
    int32_t stop_dist = static_cast<int32_t>((v * v) / (2.0f * STEPPER_HALT_DECEL)) + 1;
    const int32_t to_go = s.distanceToGo();
    if ((to_go > 0) == (v > 0.0f) && abs(to_go) < stop_dist) {
      stop_dist = abs(to_go);
    }
    s.moveTo(s.currentPosition() + (v > 0.0f ? stop_dist : -stop_dist));
  }

  if (!instant) {
    bool moving = true;
    while (moving) {
      moving = false;
      for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
        if (active[i] && steppers[i].run()) {
          moving = true;
        }
      }
    }
    for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
      if (active[i]) {
        steppers[i].setAcceleration(g_accel);
      }
    }
  }
}

// Drives active motors to absolute targets. Halts on pause, resumes to the same targets.
bool move_blocking(const bool active[], const int32_t target[]) {
  if (!run_wait_resume()) {
    return false;
  }

  prime_dir(active, target);
  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    if (active[i]) {
      start(i, target[i]);
    }
  }

  for (;;) {
    bool moving = false;
    for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
      if (active[i] && service(i, target[i])) {
        moving = true;
      }
    }
    if (!moving) {
      return true;
    }

    if (run_halted()) {
      halt(active);
      if (!run_wait_resume()) {
        return false;
      }
      prime_dir(active, target);
      for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
        if (active[i]) {
          start(i, target[i]);
        }
      }
    }
  }
}

}  // namespace

void stepper_init() {
  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    digitalWrite(kPins[i].step, LOW);
    digitalWrite(kPins[i].dir, kPins[i].dir_invert ? HIGH : LOW);

    AccelStepper &s = steppers[i];
    s.setPinsInverted(kPins[i].dir_invert, false, false);
    s.setMinPulseWidth(STEPPER_PULSE_WIDTH_US);
    s.enableOutputs();  // step/dir pins to OUTPUT
    s.setCurrentPosition(0);
  }
  stepper_set_config(STEPPER_MAX_SPEED, STEPPER_ACCEL);
  stepper_enable(true);
}

void stepper_set_config(float speed, float acceleration) {
  g_max_speed = (speed > 0.0f) ? speed : STEPPER_MAX_SPEED;
  g_no_ramp = (acceleration == -1.0f);
  g_accel = (acceleration > 0.0f) ? acceleration : STEPPER_ACCEL;

  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    steppers[i].setMaxSpeed(g_max_speed);
    steppers[i].setAcceleration(g_accel);
  }
}

void stepper_enable(bool enabled) {
  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    io_expander_write(kPins[i].exio_en, enabled == STEPPER_EN_ACTIVE_HIGH);
  }
}

bool stepper_run_steps_blocking(uint8_t motor_number, int32_t steps, Direction direction) {
  const StepperMove move = {motor_number, steps, direction};
  return stepper_run_steps_batch_blocking(&move, 1);
}

bool stepper_run_steps_batch_blocking(const StepperMove *moves, uint8_t move_count) {
  if (moves == nullptr) {
    return true;
  }

  bool active[STEPPER_COUNT] = {};
  int32_t target[STEPPER_COUNT] = {};
  bool any = false;

  for (uint8_t m = 0; m < move_count; ++m) {
    if (!valid(moves[m].motor_number) || moves[m].steps <= 0) {
      continue;
    }
    const uint8_t i = moves[m].motor_number - 1;
    const int32_t delta = (moves[m].direction == Direction::CW) ? moves[m].steps : -moves[m].steps;
    active[i] = true;
    target[i] = steppers[i].currentPosition() + delta;
    any = true;
  }

  return any ? move_blocking(active, target) : true;
}

bool stepper_home_all_blocking() {
  bool active[STEPPER_COUNT] = {};
  const int32_t target[STEPPER_COUNT] = {};  // home = 0

  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    active[i] = steppers[i].currentPosition() != 0;
  }
  return move_blocking(active, target);
}

int32_t stepper_get_position(uint8_t motor_number) {
  return valid(motor_number) ? steppers[motor_number - 1].currentPosition() : 0;
}
