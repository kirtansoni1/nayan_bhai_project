#include "indicator.h"

#include <atomic>

#include "config.h"
#include "fault.h"

namespace {

constexpr uint32_t TICK_MS = 20;
constexpr uint32_t TASK_STACK = 2048;
constexpr UBaseType_t TASK_PRIORITY = 2;
constexpr BaseType_t TASK_CORE = 0;

std::atomic<bool> g_green{false};
std::atomic<bool> g_red{false};
TaskHandle_t g_task = nullptr;

uint8_t level(bool on) {
  return (on == LED_ACTIVE_HIGH) ? HIGH : LOW;
}

// RED state for a blink code at time t (ms) into its pattern
bool blink_on(uint8_t code, uint32_t t) {
  const uint32_t slot = ERR_BLINK_ON_MS + ERR_BLINK_OFF_MS;
  const uint32_t period = code * slot + ERR_BLINK_PAUSE_MS;
  t %= period;
  return t < code * slot && (t % slot) < ERR_BLINK_ON_MS;
}

void indicator_task(void *) {
  uint8_t shown = 0;
  uint32_t pattern_start = 0;

  for (;;) {
    const uint8_t code = fault_code();
    const uint32_t now = millis();

    if (code != shown) {
      shown = code;
      pattern_start = now;  // restart so the blink count is always complete
    }

    const bool red = (code == 0) ? g_red.load() : blink_on(code, now - pattern_start);
    digitalWrite(PIN_LED_GREEN, level(g_green.load()));
    digitalWrite(PIN_LED_RED, level(red));

    vTaskDelay(pdMS_TO_TICKS(TICK_MS));
  }
}

}  // namespace

void indicator_init() {
  digitalWrite(PIN_LED_GREEN, level(false));
  digitalWrite(PIN_LED_RED, level(false));
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);

  if (g_task == nullptr) {
    xTaskCreatePinnedToCore(indicator_task, "leds", TASK_STACK, nullptr, TASK_PRIORITY, &g_task, TASK_CORE);
  }
}

void indicator_set(bool green, bool red) {
  g_green.store(green);
  g_red.store(red);
}
