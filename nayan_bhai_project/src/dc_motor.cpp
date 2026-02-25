#include "dc_motor.h"
#include "main.h"

namespace {
constexpr uint8_t CH_DC3000_R = 0;
constexpr uint8_t CH_DC3000_L = 1;
constexpr uint8_t CH_DC1_300_R = 2;
constexpr uint8_t CH_DC1_300_L = 3;
constexpr uint8_t CH_DC2_300_R = 4;
constexpr uint8_t CH_DC2_300_L = 5;

struct DcRuntime {
  bool running;
  bool timedRunActive;
  uint32_t timedRunEndMs;
  uint8_t speed;
  Direction direction;
  gpio_num_t pinRpwm;
  gpio_num_t pinLpwm;
  uint8_t chRpwm;
  uint8_t chLpwm;
};

DcRuntime dc3000 = {
    false, false, 0, 0, Direction::CW,
  PIN_DC_3000_RPWM, PIN_DC_3000_LPWM,
    CH_DC3000_R, CH_DC3000_L,
};

DcRuntime dc1_300 = {
    false, false, 0, 0, Direction::CW,
  PIN_DC1_300_RPWM, PIN_DC1_300_LPWM,
  CH_DC1_300_R, CH_DC1_300_L,
};

DcRuntime dc2_300 = {
  false, false, 0, 0, Direction::CW,
  PIN_DC2_300_RPWM, PIN_DC2_300_LPWM,
  CH_DC2_300_R, CH_DC2_300_L,
};

uint8_t clamp_speed(uint8_t speed) {
  if (speed > DC_PWM_MAX) {
    return DC_PWM_MAX;
  }
  return speed;
}

DcRuntime *motor_from_id(DcMotorId id) {
  switch (id) {
    case DcMotorId::M3000:
      return &dc3000;
    case DcMotorId::M1_300:
      return &dc1_300;
    case DcMotorId::M2_300:
      return &dc2_300;
    default:
      return nullptr;
  }
}

bool is_timed_motion_complete(const DcRuntime &motor) {
  return !motor.timedRunActive && !motor.running;
}

void set_motor_enable(const DcRuntime &motor, bool enabled) {
  if (&motor == &dc3000) {
    digitalWrite(static_cast<uint8_t>(PIN_DC_3000_EN), enabled ? HIGH : LOW);
  } else {
    digitalWrite(static_cast<uint8_t>(PIN_DC_300_EN), enabled ? HIGH : LOW);
  }
}

void write_motor_outputs(DcRuntime &motor) {
  if (!motor.running || motor.speed == 0) {
    ledcWrite(motor.chRpwm, 0);
    ledcWrite(motor.chLpwm, 0);
    return;
  }

  if (motor.direction == Direction::CW) {
    ledcWrite(motor.chRpwm, motor.speed);
    ledcWrite(motor.chLpwm, 0);
  } else {
    ledcWrite(motor.chRpwm, 0);
    ledcWrite(motor.chLpwm, motor.speed);
  }
}

void stop_motor(DcRuntime &motor) {
  motor.running = false;
  motor.timedRunActive = false;
  motor.speed = 0;
  write_motor_outputs(motor);
  set_motor_enable(motor, false);
}

void run_motor(DcRuntime &motor, uint8_t speed, Direction direction) {
  set_motor_enable(motor, true);
  motor.running = true;
  motor.timedRunActive = false;
  motor.speed = clamp_speed(speed);
  motor.direction = direction;
  write_motor_outputs(motor);
}

void run_motor_ms(DcRuntime &motor, uint32_t time_ms, uint8_t speed, Direction direction) {
  if (time_ms == 0) {
    stop_motor(motor);
    return;
  }

  set_motor_enable(motor, true);
  motor.running = true;
  motor.timedRunActive = true;
  motor.timedRunEndMs = millis() + time_ms;
  motor.speed = clamp_speed(speed);
  motor.direction = direction;
  write_motor_outputs(motor);
}
}  // namespace

void dc_motor_init() {
  pinMode(static_cast<uint8_t>(PIN_DC_3000_EN), OUTPUT);
  pinMode(static_cast<uint8_t>(PIN_DC_300_EN), OUTPUT);

  ledcSetup(dc3000.chRpwm, DC_PWM_FREQ_HZ, DC_PWM_BITS);
  ledcSetup(dc3000.chLpwm, DC_PWM_FREQ_HZ, DC_PWM_BITS);
  ledcSetup(dc1_300.chRpwm, DC_PWM_FREQ_HZ, DC_PWM_BITS);
  ledcSetup(dc1_300.chLpwm, DC_PWM_FREQ_HZ, DC_PWM_BITS);
  ledcSetup(dc2_300.chRpwm, DC_PWM_FREQ_HZ, DC_PWM_BITS);
  ledcSetup(dc2_300.chLpwm, DC_PWM_FREQ_HZ, DC_PWM_BITS);

  ledcAttachPin(static_cast<uint8_t>(dc3000.pinRpwm), dc3000.chRpwm);
  ledcAttachPin(static_cast<uint8_t>(dc3000.pinLpwm), dc3000.chLpwm);
  ledcAttachPin(static_cast<uint8_t>(dc1_300.pinRpwm), dc1_300.chRpwm);
  ledcAttachPin(static_cast<uint8_t>(dc1_300.pinLpwm), dc1_300.chLpwm);
  ledcAttachPin(static_cast<uint8_t>(dc2_300.pinRpwm), dc2_300.chRpwm);
  ledcAttachPin(static_cast<uint8_t>(dc2_300.pinLpwm), dc2_300.chLpwm);

  digitalWrite(static_cast<uint8_t>(PIN_DC_3000_EN), HIGH);
  digitalWrite(static_cast<uint8_t>(PIN_DC_300_EN), HIGH);

  stop_motor(dc3000);
  stop_motor(dc1_300);
  stop_motor(dc2_300);
}

void dc_service() {
  const uint32_t now = millis();

  if (dc3000.timedRunActive && static_cast<int32_t>(now - dc3000.timedRunEndMs) >= 0) {
    stop_motor(dc3000);
  }
  if (dc1_300.timedRunActive && static_cast<int32_t>(now - dc1_300.timedRunEndMs) >= 0) {
    stop_motor(dc1_300);
  }
  if (dc2_300.timedRunActive && static_cast<int32_t>(now - dc2_300.timedRunEndMs) >= 0) {
    stop_motor(dc2_300);
  }
}

void dc_3000_run(uint8_t speed, Direction direction) {
  run_motor(dc3000, speed, direction);
}

void dc_3000_run_ms(uint32_t time_ms, uint8_t speed, Direction direction) {
  run_motor_ms(dc3000, time_ms, speed, direction);
}

void dc_3000_run_ms_blocking(uint32_t time_ms, uint8_t speed, Direction direction) {
  dc_3000_run_ms(time_ms, speed, direction);

  for (;;) {
    if (g_paused) {
      // Capture remaining run time before stopping
      const int32_t left = static_cast<int32_t>(dc3000.timedRunEndMs - millis());
      const uint32_t remaining_ms = (dc3000.timedRunActive && left > 0) ? static_cast<uint32_t>(left) : 0;
      dc_3000_stop();
      while (g_paused) delay(10);
      if (remaining_ms > 0) {
        dc_3000_run_ms(remaining_ms, speed, direction);
      } else {
        break;
      }
    }

    dc_service();

    if (is_timed_motion_complete(dc3000)) {
      break;
    }

    delay(0);
  }
}

void dc_3000_stop() {
  stop_motor(dc3000);
}

void dc1_300_run(uint8_t speed, Direction direction) {
  run_motor(dc1_300, speed, direction);
}

void dc1_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction) {
  run_motor_ms(dc1_300, time_ms, speed, direction);
}

void dc1_300_run_ms_blocking(uint32_t time_ms, uint8_t speed, Direction direction) {
  dc1_300_run_ms(time_ms, speed, direction);

  for (;;) {
    if (g_paused) {
      // Capture remaining run time before stopping
      const int32_t left = static_cast<int32_t>(dc1_300.timedRunEndMs - millis());
      const uint32_t remaining_ms = (dc1_300.timedRunActive && left > 0) ? static_cast<uint32_t>(left) : 0;
      dc1_300_stop();
      while (g_paused) delay(10);
      if (remaining_ms > 0) {
        dc1_300_run_ms(remaining_ms, speed, direction);
      } else {
        break;
      }
    }

    dc_service();

    if (is_timed_motion_complete(dc1_300)) {
      break;
    }

    delay(0);
  }
}

void dc1_300_stop() {
  stop_motor(dc1_300);
}

void dc2_300_run(uint8_t speed, Direction direction) {
  run_motor(dc2_300, speed, direction);
}

void dc2_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction) {
  run_motor_ms(dc2_300, time_ms, speed, direction);
}

void dc2_300_run_ms_blocking(uint32_t time_ms, uint8_t speed, Direction direction) {
  dc2_300_run_ms(time_ms, speed, direction);

  for (;;) {
    if (g_paused) {
      // Capture remaining run time before stopping
      const int32_t left = static_cast<int32_t>(dc2_300.timedRunEndMs - millis());
      const uint32_t remaining_ms = (dc2_300.timedRunActive && left > 0) ? static_cast<uint32_t>(left) : 0;
      dc2_300_stop();
      while (g_paused) delay(10);
      if (remaining_ms > 0) {
        dc2_300_run_ms(remaining_ms, speed, direction);
      } else {
        break;
      }
    }

    dc_service();

    if (is_timed_motion_complete(dc2_300)) {
      break;
    }

    delay(0);
  }
}

void dc2_300_stop() {
  stop_motor(dc2_300);
}

void dc_run_ms_batch_blocking(const DcTimedMove *moves, uint8_t move_count) {
  if (moves == nullptr || move_count == 0) {
    return;
  }

  bool hasValidMove = false;

  for (uint8_t i = 0; i < move_count; ++i) {
    DcRuntime *motor = motor_from_id(moves[i].motor);
    if (motor == nullptr) {
      continue;
    }

    hasValidMove = true;
    run_motor_ms(*motor, moves[i].time_ms, moves[i].speed, moves[i].direction);
  }

  if (!hasValidMove) {
    return;
  }

  for (;;) {
    if (g_paused) {
      // Capture remaining time for each motor before stopping
      uint32_t remaining_ms[3] = {};
      for (uint8_t i = 0; i < move_count; ++i) {
        DcRuntime *motor = motor_from_id(moves[i].motor);
        if (motor == nullptr || !motor->timedRunActive) continue;
        const int32_t left = static_cast<int32_t>(motor->timedRunEndMs - millis());
        remaining_ms[static_cast<uint8_t>(moves[i].motor)] = (left > 0) ? static_cast<uint32_t>(left) : 0;
      }
      dc_stop_all();
      while (g_paused) delay(10);
      // Resume each motor with its remaining time
      for (uint8_t i = 0; i < move_count; ++i) {
        DcRuntime *motor = motor_from_id(moves[i].motor);
        if (motor == nullptr) continue;
        const uint32_t rem = remaining_ms[static_cast<uint8_t>(moves[i].motor)];
        if (rem > 0) {
          run_motor_ms(*motor, rem, moves[i].speed, moves[i].direction);
        }
      }
    }

    dc_service();

    bool allComplete = true;
    for (uint8_t i = 0; i < move_count; ++i) {
      DcRuntime *motor = motor_from_id(moves[i].motor);
      if (motor == nullptr) {
        continue;
      }

      if (!is_timed_motion_complete(*motor)) {
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

void dc_stop_all() {
  stop_motor(dc3000);
  stop_motor(dc1_300);
  stop_motor(dc2_300);
}
