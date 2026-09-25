#include "io_expander.h"

#include <TCA9555.h>
#include <Wire.h>

#include "config.h"
#include "fault.h"
#include "logger.h"

namespace {

TCA9535 expander(IO_EXPANDER_ADDR, &Wire);

// Shadow of both output registers, so a pin change is a single port write
uint16_t g_out = 0;
uint16_t g_out_mask = 0;  // 1 = pin used as output

// Stepper driver enables: set once at boot, writes to them are refused
constexpr uint16_t kFixedMask = static_cast<uint16_t>((1u << EXIO_S_M1_EN) | (1u << EXIO_S_M2_EN) |
                                                      (1u << EXIO_S_M3_EN));

uint8_t port_byte(uint16_t v, uint8_t port) {
  return static_cast<uint8_t>(port == 0 ? (v & 0xFF) : (v >> 8));
}

void set_shadow(uint8_t pin, bool level) {
  const uint16_t bit = static_cast<uint16_t>(1u << pin);
  g_out = level ? (g_out | bit) : (g_out & ~bit);
}

void add_output(uint8_t pin, bool level) {
  g_out_mask |= static_cast<uint16_t>(1u << pin);
  set_shadow(pin, level);
}

bool write_port(uint8_t port) {
  return expander.write8(port, port_byte(g_out, port));
}

// Output register first, then direction (config bit 0 = output).
// Also restores the setup after an expander power-on reset.
bool configure() {
  bool ok = write_port(0) && write_port(1);
  ok = ok && expander.pinMode8(0, static_cast<uint8_t>(~port_byte(g_out_mask, 0)));
  ok = ok && expander.pinMode8(1, static_cast<uint8_t>(~port_byte(g_out_mask, 1)));
  return ok;
}

// Reads the real pin levels back and compares them with what was written
bool verify_port(uint8_t port) {
  const uint8_t mask = port_byte(g_out_mask, port);
  if (mask == 0) {
    return true;
  }
  const int pins = expander.read8(port);
  if (expander.lastError() != TCA9555_OK) {
    return false;
  }
  const uint8_t wrong = (static_cast<uint8_t>(pins) ^ port_byte(g_out, port)) & mask;
  if (wrong != 0) {
    LOG("IO expander port %u pins 0x%02X, expected 0x%02X, restoring\n", port,
        static_cast<uint8_t>(pins) & mask, port_byte(g_out, port) & mask);
    return false;
  }
  return true;
}

// Writes a port and proves the pins follow. On mismatch the full setup is re-applied.
bool apply_port(uint8_t port) {
  for (uint8_t attempt = 0; attempt < IO_EXPANDER_WRITE_TRIES; ++attempt) {
    const bool written = (attempt == 0) ? write_port(port) : configure();
    if (written && verify_port(port)) {
      return true;
    }
  }

  // Latched: outputs behind the expander are now in an unknown state
  fault_set(Fault::IO_EXPANDER);
  return false;
}

}  // namespace

bool io_expander_init() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);

  if (!expander.isConnected()) {
    return false;
  }

  // DC drivers disabled and solenoid off. Stepper drivers enabled here once and
  // never changed after, so they keep holding torque and position.
  g_out = 0xFFFF;  // unused pins stay inputs, register value does not matter
  g_out_mask = 0;
  add_output(EXIO_SOLENOID_RLY, !SOLENOID_ACTIVE_HIGH);
  add_output(EXIO_DC_3000_EN, !DC_EN_ACTIVE_HIGH);
  add_output(EXIO_DC1_300_EN, !DC_EN_ACTIVE_HIGH);
  add_output(EXIO_DC2_300_EN, !DC_EN_ACTIVE_HIGH);
  add_output(EXIO_S_M1_EN, STEPPER_EN_ACTIVE_HIGH);
  add_output(EXIO_S_M2_EN, STEPPER_EN_ACTIVE_HIGH);
  add_output(EXIO_S_M3_EN, STEPPER_EN_ACTIVE_HIGH);

  return configure() && verify_port(0) && verify_port(1);
}

bool io_expander_write(uint8_t pin, bool level) {
  if (pin > 15 || !(g_out_mask & (1u << pin)) || (kFixedMask & (1u << pin))) {
    return false;
  }

  set_shadow(pin, level);
  return apply_port(pin < 8 ? 0 : 1);
}

bool io_expander_check() {
  if (fault_active(Fault::IO_EXPANDER)) {
    return false;
  }
  for (uint8_t port = 0; port < 2; ++port) {
    if (!verify_port(port) && !apply_port(port)) {
      return false;
    }
  }
  return true;
}
