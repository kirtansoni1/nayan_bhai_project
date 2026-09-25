#include "stepper_motor.h"

#include <AccelStepper.h>

#include "logger.h"
#include "run_control.h"

namespace {

// AccelStepper with DM542 safe pulses. Stock AccelStepper writes DIR in the same
// instant as STEP, so a direction change can be stepped the old way and the
// position count drifts. Here DIR always settles before the pulse.
class Dm542Stepper : public AccelStepper {
 public:
  Dm542Stepper(uint8_t step_pin, uint8_t dir_pin, bool dir_invert)
      : AccelStepper(AccelStepper::DRIVER, step_pin, dir_pin, 0xff, 0xff, false),
        step_pin_(step_pin), dir_pin_(dir_pin), dir_invert_(dir_invert) {}

  // STEP low, DIR at a known level, both outputs
  void begin() {
    dir_level_ = !dir_invert_;  // CW level
    digitalWrite(step_pin_, LOW);
    digitalWrite(dir_pin_, dir_level_ ? HIGH : LOW);
    pinMode(step_pin_, OUTPUT);
    pinMode(dir_pin_, OUTPUT);
  }

 protected:
  // Called by AccelStepper for every step, after it has updated the position
  void step(long) override {
    const bool level = (_direction == DIRECTION_CW) != dir_invert_;
    if (level != dir_level_) {
      digitalWrite(dir_pin_, level ? HIGH : LOW);
      dir_level_ = level;
      delayMicroseconds(STEPPER_DIR_SETUP_US);
    }
    digitalWrite(step_pin_, HIGH);
    delayMicroseconds(STEPPER_PULSE_WIDTH_US);
    digitalWrite(step_pin_, LOW);
  }

 private:
  uint8_t step_pin_;
  uint8_t dir_pin_;
  bool dir_invert_;
  bool dir_level_ = false;
};

Dm542Stepper steppers[STEPPER_COUNT] = {
  Dm542Stepper(PIN_S_M1_STEP, PIN_S_M1_DIR, S_M1_DIR_INVERT),
  Dm542Stepper(PIN_S_M2_STEP, PIN_S_M2_DIR, S_M2_DIR_INVERT),
  Dm542Stepper(PIN_S_M3_STEP, PIN_S_M3_DIR, S_M3_DIR_INVERT),
};

float g_max_speed = STEPPER_MAX_SPEED;
float g_accel = STEPPER_ACCEL;
bool g_no_ramp = false;

bool valid(uint8_t motor_number) {
  return motor_number >= 1 && motor_number <= STEPPER_COUNT;
}

// Steps needed to stop from speed v at deceleration a
int32_t stop_steps(float v, float a) {
  return static_cast<int32_t>((v * v) / (2.0f * a)) + 1;
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

// Brings active motors to rest without reversing and without passing the move target.
// Position stays tracked, so the move can resume.
void halt(const bool active[]) {
  bool braking = false;

  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    if (!active[i]) {
      continue;
    }
    AccelStepper &s = steppers[i];
    const float v = s.speed();

    // Stop dead: no ramp mode, or slow enough to stop within 2 steps
    if (g_no_ramp || STEPPER_HALT_DECEL <= 0.0f || v == 0.0f || stop_steps(v, STEPPER_HALT_DECEL) <= 2) {
      s.setCurrentPosition(s.currentPosition());  // zero speed, keep position
      continue;
    }

    braking = true;
    const int32_t to_go = s.distanceToGo();
    if ((to_go > 0) == (v > 0.0f) && abs(to_go) <= stop_steps(v, fminf(g_accel, STEPPER_HALT_DECEL))) {
      continue;  // already braking onto its target, let it finish normally
    }

    // Retargeting inside the stop distance makes AccelStepper overshoot and reverse,
    // so the new target is always the full stop distance ahead
    const int32_t dist = stop_steps(v, STEPPER_HALT_DECEL);
    s.setAcceleration(STEPPER_HALT_DECEL);
    s.moveTo(s.currentPosition() + (v > 0.0f ? dist : -dist));
  }

  if (!braking) {
    return;
  }

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

// Measures the longest gap between step loop passes. A long gap means pulses were
// late and the motor may have lost steps. Only built when logs are on.
struct LoopTimer {
#if LOG_ENABLED
  uint32_t last = micros();
  uint32_t worst = 0;
  void tick() {
    const uint32_t now = micros();
    worst = max(worst, now - last);
    last = now;
  }
  void restart() { last = micros(); }
  void report(const bool active[]) const {
    if (worst < STEPPER_GAP_WARN_US) {
      return;
    }
    LOG("Step timing gap %lu us on", static_cast<unsigned long>(worst));
    for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
      if (active[i]) {
        LOG(" S%u", i + 1);
      }
    }
    LOGLN("");
  }
#else
  void tick() {}
  void restart() {}
  void report(const bool *) const {}
#endif
};

// Drives active motors to absolute targets. Halts on pause, resumes to the same targets.
bool move_blocking(const bool active[], const int32_t target[]) {
  if (!run_wait_resume()) {
    return false;
  }

  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    if (active[i]) {
      start(i, target[i]);
    }
  }

  LoopTimer timer;
  for (;;) {
    timer.tick();
    bool moving = false;
    for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
      if (active[i] && service(i, target[i])) {
        moving = true;
      }
    }
    if (!moving) {
      timer.report(active);
      return true;
    }

    if (run_halted()) {
      halt(active);
      if (!run_wait_resume()) {
        timer.report(active);
        return false;
      }
      for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
        if (active[i]) {
          start(i, target[i]);
        }
      }
      timer.restart();  // time spent paused is not a step gap
    }
  }
}

}  // namespace

void stepper_init() {
  for (uint8_t i = 0; i < STEPPER_COUNT; ++i) {
    steppers[i].begin();
    steppers[i].setCurrentPosition(0);
  }
  stepper_set_config(STEPPER_MAX_SPEED, STEPPER_ACCEL);
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
