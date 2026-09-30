#include "io_expander.h"

#include <TCA9555.h>
#include <Wire.h>

#include "config.h"
#include "fault.h"

namespace {

TCA9535 expander(IO_EXPANDER_ADDR, &Wire);

// Shadow of both output registers, so a pin change is a single port write
uint16_t g_out = 0;
uint16_t g_out_mask = 0;  // 1 = pin used as output

void set_shadow(uint8_t pin, bool level) {
  const uint16_t bit = static_cast<uint16_t>(1u << pin);
  g_out = level ? (g_out | bit) : (g_out & ~bit);
}

void add_output(uint8_t pin, bool level) {
  g_out_mask |= static_cast<uint16_t>(1u << pin);
  set_shadow(pin, level);
}

bool write_port(uint8_t port) {
  const uint8_t value = static_cast<uint8_t>(port == 0 ? (g_out & 0xFF) : (g_out >> 8));
  return expander.write8(port, value);
}

}  // namespace

bool io_expander_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);

  if (!expander.isConnected()) {
    return false;
  }

  // Safe levels: every driver disabled, solenoid off
  g_out = 0xFFFF;  // unused pins stay inputs, register value does not matter
  g_out_mask = 0;
  add_output(EXIO_SOLENOID_RLY, !SOLENOID_ACTIVE_HIGH);
  add_output(EXIO_DC_3000_EN, !DC_EN_ACTIVE_HIGH);
  add_output(EXIO_DC1_300_EN, !DC_EN_ACTIVE_HIGH);
  add_output(EXIO_DC2_300_EN, !DC_EN_ACTIVE_HIGH);
  add_output(EXIO_S_M1_EN, !STEPPER_EN_ACTIVE_HIGH);
  add_output(EXIO_S_M2_EN, !STEPPER_EN_ACTIVE_HIGH);
  add_output(EXIO_S_M3_EN, !STEPPER_EN_ACTIVE_HIGH);

  // Output register first, then direction (config bit 0 = output)
  bool ok = write_port(0) && write_port(1);
  ok = ok && expander.pinMode8(0, static_cast<uint8_t>(~g_out_mask & 0xFF));
  ok = ok && expander.pinMode8(1, static_cast<uint8_t>(~g_out_mask >> 8));
  return ok;
}

bool io_expander_write(uint8_t pin, bool level) {
  if (pin > 15 || !(g_out_mask & (1u << pin))) {
    return false;
  }

  set_shadow(pin, level);
  for (uint8_t attempt = 0; attempt < IO_EXPANDER_WRITE_TRIES; ++attempt) {
    if (write_port(pin < 8 ? 0 : 1)) {
      return true;
    }
  }

  // Latched: outputs behind the expander are now in an unknown state
  fault_set(Fault::IO_EXPANDER);
  return false;
}
