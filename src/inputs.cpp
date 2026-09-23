#include "inputs.h"

#include <atomic>

#include "config.h"

namespace {

struct InputDef {
  uint8_t pin;
  bool active_low;
  uint32_t debounce_ms;
  const char *name;
};

const InputDef kInputs[INPUT_COUNT] = {
  {PIN_BTN_START, BTN_ACTIVE_LOW, BTN_DEBOUNCE_MS, "START"},
  {PIN_BTN_STOP, BTN_ACTIVE_LOW, BTN_DEBOUNCE_MS, "STOP"},
  {PIN_BTN_HOME, BTN_ACTIVE_LOW, BTN_DEBOUNCE_MS, "HOME"},
  {PIN_SEN_DET, SENSOR_ACTIVE_LOW, SENSOR_DEBOUNCE_MS, "SENSOR"},
};

constexpr uint32_t TASK_STACK = 4096;
constexpr UBaseType_t TASK_PRIORITY = 5;
constexpr BaseType_t TASK_CORE = 0;  // machine loop runs on core 1

TaskHandle_t g_task = nullptr;
InputCallback g_callback = nullptr;
std::atomic<bool> g_stable[INPUT_COUNT];

bool read_active(uint8_t i) {
  return (digitalRead(kInputs[i].pin) == LOW) == kInputs[i].active_low;
}

// Edge ISR: only flags which input moved, debounce happens in the task
void IRAM_ATTR on_edge(void *arg) {
  BaseType_t woken = pdFALSE;
  xTaskNotifyFromISR(g_task, reinterpret_cast<uint32_t>(arg), eSetBits, &woken);
  if (woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

// Accepts a new level only after the line has been quiet for debounce_ms
void input_task(void *) {
  bool pending[INPUT_COUNT] = {};
  uint32_t deadline[INPUT_COUNT] = {};

  for (;;) {
    TickType_t wait = portMAX_DELAY;
    const uint32_t now = millis();
    for (uint8_t i = 0; i < INPUT_COUNT; ++i) {
      if (pending[i]) {
        const int32_t left = static_cast<int32_t>(deadline[i] - now);
        const TickType_t t = (left > 0) ? pdMS_TO_TICKS(left) + 1 : 1;
        wait = min(wait, t);
      }
    }

    uint32_t bits = 0;
    xTaskNotifyWait(0, UINT32_MAX, &bits, wait);

    const uint32_t t = millis();
    for (uint8_t i = 0; i < INPUT_COUNT; ++i) {
      if (bits & (1u << i)) {
        pending[i] = true;
        deadline[i] = t + kInputs[i].debounce_ms;
      }
    }

    for (uint8_t i = 0; i < INPUT_COUNT; ++i) {
      if (!pending[i] || static_cast<int32_t>(t - deadline[i]) < 0) {
        continue;
      }
      pending[i] = false;
      const bool active = read_active(i);
      if (active != g_stable[i].load()) {
        g_stable[i].store(active);
        if (g_callback != nullptr) {
          g_callback(static_cast<InputId>(i), active);
        }
      }
    }
  }
}

}  // namespace

void inputs_init(InputCallback callback) {
  if (g_task != nullptr) {
    return;
  }
  g_callback = callback;

  for (uint8_t i = 0; i < INPUT_COUNT; ++i) {
    pinMode(kInputs[i].pin, kInputs[i].active_low ? INPUT_PULLUP : INPUT_PULLDOWN);
  }
  delay(2);  // let pull resistors settle before the first read
  for (uint8_t i = 0; i < INPUT_COUNT; ++i) {
    g_stable[i].store(read_active(i));
  }

  xTaskCreatePinnedToCore(input_task, "inputs", TASK_STACK, nullptr, TASK_PRIORITY, &g_task, TASK_CORE);

  for (uint8_t i = 0; i < INPUT_COUNT; ++i) {
    attachInterruptArg(digitalPinToInterrupt(kInputs[i].pin), on_edge,
                       reinterpret_cast<void *>(1u << i), CHANGE);
  }
}

bool input_is_active(InputId id) {
  const uint8_t i = static_cast<uint8_t>(id);
  return i < INPUT_COUNT && g_stable[i].load();
}

const char *input_name(InputId id) {
  const uint8_t i = static_cast<uint8_t>(id);
  return i < INPUT_COUNT ? kInputs[i].name : "?";
}
