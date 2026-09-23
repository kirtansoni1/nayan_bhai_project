#include "dc_motor.h"

#include <esp_arduino_version.h>

#include "io_expander.h"
#include "run_control.h"

namespace {

struct DcChannel {
  uint8_t pin_rpwm;
  uint8_t pin_lpwm;
  uint8_t ch_rpwm;  // LEDC channel, used on core 2.x only
  uint8_t ch_lpwm;
  uint8_t exio_en;
  bool dir_invert;
};

const DcChannel kMotors[DC_MOTOR_COUNT] = {
  {PIN_DC1_300_RPWM, PIN_DC1_300_LPWM, 0, 1, EXIO_DC1_300_EN, DC1_300_DIR_INVERT},
  {PIN_DC2_300_RPWM, PIN_DC2_300_LPWM, 2, 3, EXIO_DC2_300_EN, DC2_300_DIR_INVERT},
  {PIN_DC_3000_RPWM, PIN_DC_3000_LPWM, 4, 5, EXIO_DC_3000_EN, DC_3000_DIR_INVERT},
};

bool g_enabled[DC_MOTOR_COUNT] = {};

bool valid(DcMotorId motor) {
  return static_cast<uint8_t>(motor) < DC_MOTOR_COUNT;
}

void pwm_setup(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcAttach(pin, DC_PWM_FREQ_HZ, DC_PWM_BITS);
#else
  ledcSetup(channel, DC_PWM_FREQ_HZ, DC_PWM_BITS);
  ledcAttachPin(pin, channel);
#endif
}

void pwm_write(uint8_t pin, uint8_t channel, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

void set_enable(uint8_t index, bool on) {
  if (g_enabled[index] == on) {
    return;  // skip redundant I2C writes
  }
  if (io_expander_write(kMotors[index].exio_en, on == DC_EN_ACTIVE_HIGH)) {
    g_enabled[index] = on;
  }
}

void write_outputs(uint8_t index, uint8_t speed, Direction direction) {
  const DcChannel &m = kMotors[index];
  const bool use_r = (direction == Direction::CW) != m.dir_invert;
  const uint8_t duty = min(speed, DC_PWM_MAX);

  // Low side first so both half bridges never drive at once
  if (use_r) {
    pwm_write(m.pin_lpwm, m.ch_lpwm, 0);
    pwm_write(m.pin_rpwm, m.ch_rpwm, duty);
  } else {
    pwm_write(m.pin_rpwm, m.ch_rpwm, 0);
    pwm_write(m.pin_lpwm, m.ch_lpwm, duty);
  }
}

void start(uint8_t index, uint8_t speed, Direction direction) {
  set_enable(index, true);
  write_outputs(index, speed, direction);
}

void halt(uint8_t index) {
  const DcChannel &m = kMotors[index];
  pwm_write(m.pin_rpwm, m.ch_rpwm, 0);
  pwm_write(m.pin_lpwm, m.ch_lpwm, 0);
  set_enable(index, false);
}

}  // namespace

void dc_motor_init() {
  for (uint8_t i = 0; i < DC_MOTOR_COUNT; ++i) {
    pwm_setup(kMotors[i].pin_rpwm, kMotors[i].ch_rpwm);
    pwm_setup(kMotors[i].pin_lpwm, kMotors[i].ch_lpwm);
    g_enabled[i] = true;  // force the first disable write
    halt(i);
  }
}

void dc_run(DcMotorId motor, uint8_t speed, Direction direction) {
  if (valid(motor)) {
    start(static_cast<uint8_t>(motor), speed, direction);
  }
}

void dc_stop(DcMotorId motor) {
  if (valid(motor)) {
    halt(static_cast<uint8_t>(motor));
  }
}

void dc_stop_all() {
  for (uint8_t i = 0; i < DC_MOTOR_COUNT; ++i) {
    halt(i);
  }
}

bool dc_run_ms_blocking(DcMotorId motor, uint32_t time_ms, uint8_t speed, Direction direction) {
  const DcTimedMove move = {motor, time_ms, speed, direction};
  return dc_run_ms_batch_blocking(&move, 1);
}

bool dc_run_ms_batch_blocking(const DcTimedMove *moves, uint8_t move_count) {
  if (moves == nullptr || move_count == 0) {
    return true;
  }
  if (!run_wait_resume()) {
    return false;
  }

  uint32_t remaining_us[DC_MOTOR_COUNT] = {};
  uint32_t started_us[DC_MOTOR_COUNT] = {};
  const DcTimedMove *active[DC_MOTOR_COUNT] = {};

  for (uint8_t i = 0; i < move_count; ++i) {
    if (!valid(moves[i].motor) || moves[i].time_ms == 0) {
      continue;
    }
    const uint8_t idx = static_cast<uint8_t>(moves[i].motor);
    active[idx] = &moves[i];
    remaining_us[idx] = moves[i].time_ms * 1000UL;
  }

  const uint32_t now = micros();
  for (uint8_t idx = 0; idx < DC_MOTOR_COUNT; ++idx) {
    if (active[idx] != nullptr) {
      started_us[idx] = now;
      start(idx, active[idx]->speed, active[idx]->direction);
    }
  }

  for (;;) {
    bool all_done = true;
    const uint32_t t = micros();

    for (uint8_t idx = 0; idx < DC_MOTOR_COUNT; ++idx) {
      if (active[idx] == nullptr) {
        continue;
      }
      if (t - started_us[idx] >= remaining_us[idx]) {
        halt(idx);
        active[idx] = nullptr;
      } else {
        all_done = false;
      }
    }

    if (all_done) {
      return true;
    }

    if (run_halted()) {
      // Save time left, stop, then continue from there on resume
      const uint32_t t_halt = micros();
      for (uint8_t idx = 0; idx < DC_MOTOR_COUNT; ++idx) {
        if (active[idx] == nullptr) {
          continue;
        }
        const uint32_t ran = t_halt - started_us[idx];
        remaining_us[idx] = (ran < remaining_us[idx]) ? remaining_us[idx] - ran : 0;
        halt(idx);
      }

      if (!run_wait_resume()) {
        return false;
      }

      const uint32_t t_resume = micros();
      for (uint8_t idx = 0; idx < DC_MOTOR_COUNT; ++idx) {
        if (active[idx] != nullptr) {
          started_us[idx] = t_resume;
          start(idx, active[idx]->speed, active[idx]->direction);
        }
      }
    }

    delay(1);
  }
}
