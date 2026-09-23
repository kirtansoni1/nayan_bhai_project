#include "solenoid.h"

#include "config.h"
#include "io_expander.h"

namespace {
bool g_on = false;
}

void solenoid_set(bool on) {
  g_on = on;
  io_expander_write(EXIO_SOLENOID_RLY, on == SOLENOID_ACTIVE_HIGH);
}

bool solenoid_is_on() {
  return g_on;
}
