#include "button_matrix.h"

#include <Keypad.h>
#include "defines.h"

namespace {

// Physical keypad character map.
// The Keypad library drives our COL pins (C0..C3) one-at-a-time LOW and reads our ROW pins (R0..R3).
//   C0 low -> R0=BTN1   R1=BTN4  R2=BTN7   R3=BTNSTAR
//   C1 low -> R0=BTN2   R1=BTN5  R2=BTN8   R3=BTN0
//   C2 low -> R0=BTN3   R1=BTN6  R2=BTN9   R3=BTNHASH
//   C3 low -> R0=BTNA   R1=BTNB  R2=BTNC   R3=BTND
constexpr byte LIBROWS = 4;
constexpr byte LIBCOLS = 4;

char keymap[LIBROWS][LIBCOLS] = {
  {'1', '2', '3', 'A'},  // L1: BTN1, BTN2, BTN3, BTNA
  {'4', '5', '6', 'B'},  // L2: BTN4, BTN5, BTN6, BTNB
  {'7', '8', '9', 'C'},  // L3: BTN7, BTN8, BTN9, BTNC
  {'*', '0', '#', 'D'},  // L4: BTNSTAR, BTN0, BTNHASH, BTND
};

byte libRowPins[LIBROWS] = {
  static_cast<byte>(PIN_BTN_C0),
  static_cast<byte>(PIN_BTN_C1),
  static_cast<byte>(PIN_BTN_C2),
  static_cast<byte>(PIN_BTN_C3),
};

byte libColPins[LIBCOLS] = {
  static_cast<byte>(PIN_BTN_R0),
  static_cast<byte>(PIN_BTN_R1),
  static_cast<byte>(PIN_BTN_R2),
  static_cast<byte>(PIN_BTN_R3),
};

Keypad keypad = Keypad(makeKeymap(keymap), libRowPins, libColPins, LIBROWS, LIBCOLS);

ButtonEventCallback g_callback = nullptr;
TaskHandle_t buttonTaskHandle = nullptr;

ButtonId char_to_button_id(char c) {
  switch (c) {
    case '1': return ButtonId::BTN1;
    case '2': return ButtonId::BTN2;
    case '3': return ButtonId::BTN3;
    case 'A': return ButtonId::BTNA;
    case '4': return ButtonId::BTN4;
    case '5': return ButtonId::BTN5;
    case '6': return ButtonId::BTN6;
    case 'B': return ButtonId::BTNB;
    case '7': return ButtonId::BTN7;
    case '8': return ButtonId::BTN8;
    case '9': return ButtonId::BTN9;
    case 'C': return ButtonId::BTNC;
    case '*': return ButtonId::BTNSTAR;
    case '0': return ButtonId::BTN0;
    case '#': return ButtonId::BTNHASH;
    case 'D': return ButtonId::BTND;
    default:  return ButtonId::UNKNOWN;
  }
}

void button_scan_task(void* parameter) {
  (void)parameter;

  for (;;) {
    if (keypad.getKeys() && g_callback != nullptr) {
      for (uint8_t i = 0; i < LIST_MAX; ++i) {
        const Key& k = keypad.key[i];

        if (!k.stateChanged) {
          continue;
        }

        // Only fire on PRESSED and RELEASED - ignore HOLD/IDLE
        if (k.kstate != PRESSED && k.kstate != RELEASED) {
          continue;
        }

        const ButtonId id = char_to_button_id(k.kchar);
        if (id == ButtonId::UNKNOWN) {
          continue;
        }

        const ButtonState state = (k.kstate == PRESSED) ? ButtonState::PRESSED : ButtonState::RELEASED;
        g_callback(ButtonEvent{id, state});
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // Poll every 10ms - reliable and responsive
  }
}

}  // namespace

// --- Public API ---

const char* button_name(ButtonId id) {
  switch (id) {
    case ButtonId::BTN1:     return "BTN1";
    case ButtonId::BTN2:     return "BTN2";
    case ButtonId::BTN3:     return "BTN3";
    case ButtonId::BTNA:     return "BTNA";
    case ButtonId::BTN4:     return "BTN4";
    case ButtonId::BTN5:     return "BTN5";
    case ButtonId::BTN6:     return "BTN6";
    case ButtonId::BTNB:     return "BTNB";
    case ButtonId::BTN7:     return "BTN7";
    case ButtonId::BTN8:     return "BTN8";
    case ButtonId::BTN9:     return "BTN9";
    case ButtonId::BTNC:     return "BTNC";
    case ButtonId::BTNSTAR:  return "BTNSTAR";
    case ButtonId::BTN0:     return "BTN0";
    case ButtonId::BTNHASH:  return "BTNHASH";
    case ButtonId::BTND:     return "BTND";
    default:                 return "UNKNOWN";
  }
}

void button_matrix_init(ButtonEventCallback callback) {
  g_callback = callback;
  keypad.setDebounceTime(KEY_DEBOUNCE_MS);
  keypad.setHoldTime(1000);

  if (buttonTaskHandle == nullptr) {
    xTaskCreatePinnedToCore(button_scan_task, "button_task", 4096, nullptr, 4, &buttonTaskHandle, 0);
  }
}

