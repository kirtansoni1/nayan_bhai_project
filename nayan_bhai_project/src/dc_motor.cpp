#include "dc_motor.h"

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

portMUX_TYPE dcMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t dcTaskHandle = nullptr;

uint8_t clamp_speed(uint8_t speed) {
  if (speed > DC_PWM_MAX) {
    return DC_PWM_MAX;
  }
  return speed;
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
}

void run_motor(DcRuntime &motor, uint8_t speed, Direction direction) {
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

  motor.running = true;
  motor.timedRunActive = true;
  motor.timedRunEndMs = millis() + time_ms;
  motor.speed = clamp_speed(speed);
  motor.direction = direction;
  write_motor_outputs(motor);
}

void dc_service_task(void *parameter) {
  (void)parameter;

  for (;;) {
    const uint32_t now = millis();

    taskENTER_CRITICAL(&dcMux);
    if (dc3000.timedRunActive && static_cast<int32_t>(now - dc3000.timedRunEndMs) >= 0) {
      stop_motor(dc3000);
    }
    if (dc1_300.timedRunActive && static_cast<int32_t>(now - dc1_300.timedRunEndMs) >= 0) {
      stop_motor(dc1_300);
    }
    if (dc2_300.timedRunActive && static_cast<int32_t>(now - dc2_300.timedRunEndMs) >= 0) {
      stop_motor(dc2_300);
    }
    taskEXIT_CRITICAL(&dcMux);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
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

  taskENTER_CRITICAL(&dcMux);
  stop_motor(dc3000);
  stop_motor(dc1_300);
  stop_motor(dc2_300);
  taskEXIT_CRITICAL(&dcMux);

  if (dcTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(dc_service_task, "dc_task", 3072, nullptr, 2, &dcTaskHandle, 1);
  }
}

void dc_3000_run(uint8_t speed, Direction direction) {
  taskENTER_CRITICAL(&dcMux);
  run_motor(dc3000, speed, direction);
  taskEXIT_CRITICAL(&dcMux);
}

void dc_3000_run_ms(uint32_t time_ms, uint8_t speed, Direction direction) {
  taskENTER_CRITICAL(&dcMux);
  run_motor_ms(dc3000, time_ms, speed, direction);
  taskEXIT_CRITICAL(&dcMux);
}

void dc_3000_stop() {
  taskENTER_CRITICAL(&dcMux);
  stop_motor(dc3000);
  taskEXIT_CRITICAL(&dcMux);
}

void dc1_300_run(uint8_t speed, Direction direction) {
  taskENTER_CRITICAL(&dcMux);
  run_motor(dc1_300, speed, direction);
  taskEXIT_CRITICAL(&dcMux);
}

void dc1_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction) {
  taskENTER_CRITICAL(&dcMux);
  run_motor_ms(dc1_300, time_ms, speed, direction);
  taskEXIT_CRITICAL(&dcMux);
}

void dc1_300_stop() {
  taskENTER_CRITICAL(&dcMux);
  stop_motor(dc1_300);
  taskEXIT_CRITICAL(&dcMux);
}

void dc2_300_run(uint8_t speed, Direction direction) {
  taskENTER_CRITICAL(&dcMux);
  run_motor(dc2_300, speed, direction);
  taskEXIT_CRITICAL(&dcMux);
}

void dc2_300_run_ms(uint32_t time_ms, uint8_t speed, Direction direction) {
  taskENTER_CRITICAL(&dcMux);
  run_motor_ms(dc2_300, time_ms, speed, direction);
  taskEXIT_CRITICAL(&dcMux);
}

void dc2_300_stop() {
  taskENTER_CRITICAL(&dcMux);
  stop_motor(dc2_300);
  taskEXIT_CRITICAL(&dcMux);
}

void dc_stop_all() {
  taskENTER_CRITICAL(&dcMux);
  stop_motor(dc3000);
  stop_motor(dc1_300);
  stop_motor(dc2_300);
  taskEXIT_CRITICAL(&dcMux);
}
