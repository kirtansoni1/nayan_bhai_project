#include "button_matrix.h"

#include "defines.h"

namespace {
constexpr uint8_t ROWS = 4;
constexpr uint8_t COLS = 4;

const uint8_t rowPins[ROWS] = {
    static_cast<uint8_t>(PIN_BTN_R0),
    static_cast<uint8_t>(PIN_BTN_R1),
    static_cast<uint8_t>(PIN_BTN_R2),
    static_cast<uint8_t>(PIN_BTN_R3),
};

const uint8_t colPins[COLS] = {
    static_cast<uint8_t>(PIN_BTN_C0),
    static_cast<uint8_t>(PIN_BTN_C1),
    static_cast<uint8_t>(PIN_BTN_C2),
    static_cast<uint8_t>(PIN_BTN_C3),
};

volatile bool keyActivityFlag = false;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t buttonTaskHandle = nullptr;
MatrixEventCallback g_callback = nullptr;

bool prevPressed[ROWS][COLS] = {};
uint32_t lastChangeMs[ROWS][COLS] = {};

MatrixEvent map_event(uint8_t row, uint8_t col, bool pressed) {
  // Mapping requested by user:
  // row0_col0 -> s_m1_cw
  // row1_col0 -> s_m1_ccw
  // row0_col1 -> s_m2_cw
  // row1_col1 -> s_m2_ccw
  // row0_col2 -> s_m3_cw
  // row1_col2 -> s_m3_ccw
  if (row == 0 && col == 0) return pressed ? MatrixEvent::S_M1_CW : MatrixEvent::S_M1_STOP;
  if (row == 1 && col == 0) return pressed ? MatrixEvent::S_M1_CCW : MatrixEvent::S_M1_STOP;
  if (row == 0 && col == 1) return pressed ? MatrixEvent::S_M2_CW : MatrixEvent::S_M2_STOP;
  if (row == 1 && col == 1) return pressed ? MatrixEvent::S_M2_CCW : MatrixEvent::S_M2_STOP;
  if (row == 0 && col == 2) return pressed ? MatrixEvent::S_M3_CW : MatrixEvent::S_M3_STOP;
  if (row == 1 && col == 2) return pressed ? MatrixEvent::S_M3_CCW : MatrixEvent::S_M3_STOP;
  return MatrixEvent::NONE;
}

void IRAM_ATTR matrix_isr() {
  taskENTER_CRITICAL_ISR(&buttonMux);
  keyActivityFlag = true;
  taskEXIT_CRITICAL_ISR(&buttonMux);
}

void drive_all_cols_high() {
  for (uint8_t col = 0; col < COLS; ++col) {
    digitalWrite(colPins[col], HIGH);
  }
}

void scan_matrix_and_dispatch() {
  const uint32_t now = millis();

  for (uint8_t col = 0; col < COLS; ++col) {
    drive_all_cols_high();
    digitalWrite(colPins[col], LOW);

    for (uint8_t row = 0; row < ROWS; ++row) {
      const bool pressed = (digitalRead(rowPins[row]) == LOW);
      const bool changed = (pressed != prevPressed[row][col]);

      if (changed && (now - lastChangeMs[row][col] >= KEY_DEBOUNCE_MS)) {
        prevPressed[row][col] = pressed;
        lastChangeMs[row][col] = now;

        if (g_callback != nullptr) {
          const MatrixEvent event = map_event(row, col, pressed);
          if (event != MatrixEvent::NONE) {
            g_callback(event);
          }
        }
      }
    }
  }

  drive_all_cols_high();
}

void button_scan_task(void *parameter) {
  (void)parameter;

  for (;;) {
    bool shouldScan = false;

    taskENTER_CRITICAL(&buttonMux);
    if (keyActivityFlag) {
      shouldScan = true;
      keyActivityFlag = false;
    }
    taskEXIT_CRITICAL(&buttonMux);

    if (shouldScan) {
      scan_matrix_and_dispatch();
    }

    // Keep periodic scan as safety fallback.
    scan_matrix_and_dispatch();
    vTaskDelay(pdMS_TO_TICKS(KEY_SCAN_PERIOD_MS));
  }
}
}  // namespace

void button_matrix_init(MatrixEventCallback callback) {
  g_callback = callback;

  for (uint8_t row = 0; row < ROWS; ++row) {
    pinMode(rowPins[row], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rowPins[row]), matrix_isr, CHANGE);
  }

  for (uint8_t col = 0; col < COLS; ++col) {
    pinMode(colPins[col], OUTPUT);
  }
  drive_all_cols_high();

  if (buttonTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(button_scan_task, "button_task", 4096, nullptr, 4, &buttonTaskHandle, 0);
  }
}
