#include "fault.h"

#include <atomic>

#include "logger.h"

namespace {

std::atomic<uint8_t> g_faults{0};  // bit (code - 1) per fault

uint8_t fault_bit(Fault fault) {
  return static_cast<uint8_t>(1u << (static_cast<uint8_t>(fault) - 1));
}

const char *fault_text(Fault fault) {
  switch (fault) {
    case Fault::SENSOR_LIMIT:     return "sensor count reached";
    case Fault::HOME_REQUIRED:    return "home required";
    case Fault::UNEXPECTED_RESET: return "unexpected restart";
    case Fault::IO_EXPANDER:      return "IO expander not responding";
  }
  return "unknown";
}

}  // namespace

void fault_set(Fault fault) {
  const uint8_t prev = g_faults.fetch_or(fault_bit(fault));
  if (!(prev & fault_bit(fault))) {
    LOG("Error %u: %s\n", static_cast<uint8_t>(fault), fault_text(fault));
  }
}

void fault_clear(Fault fault) {
  g_faults.fetch_and(static_cast<uint8_t>(~fault_bit(fault)));
}

bool fault_active(Fault fault) {
  return g_faults.load() & fault_bit(fault);
}

uint8_t fault_code() {
  const uint8_t f = g_faults.load();
  for (uint8_t code = 8; code > 0; --code) {
    if (f & (1u << (code - 1))) {
      return code;
    }
  }
  return 0;
}
