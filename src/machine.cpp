#include "machine.h"

#include <atomic>

#include "config.h"
#include "fault.h"
#include "indicator.h"
#include "logger.h"
#include "run_control.h"

namespace {

std::atomic<MachineState> g_state{MachineState::STOPPED};
SemaphoreHandle_t g_lock = nullptr;

// Main task only
Activity g_activity = Activity::NONE;

// Detections counted while running, guarded by g_lock
uint32_t g_sensor_count = 0;

// Caller holds g_lock
void set_state(MachineState s) {
  g_state.store(s);
  const bool moving = (s == MachineState::RUNNING || s == MachineState::HOMING);
  indicator_set(moving, !moving);
}

// Faults that block any motion. Prints the reason.
bool motion_blocked() {
  if (fault_active(Fault::IO_EXPANDER)) {
    LOGLN("IO expander fault, restart the machine");
    return true;
  }
  return false;
}

void on_start(MachineState s) {
  if (s == MachineState::STOPPED) {
    if (fault_active(Fault::UNEXPECTED_RESET)) {
      // First press only acknowledges, operator must have checked the axes
      fault_clear(Fault::UNEXPECTED_RESET);
      LOGLN("Restart acknowledged, press START again to run");
    } else if (fault_active(Fault::HOME_REQUIRED)) {
      LOGLN("Axes not at home, press HOME first");
    } else if (!motion_blocked()) {
      fault_clear(Fault::SENSOR_LIMIT);
      set_state(MachineState::RUNNING);
      LOGLN("Starting cycle");
    }
  } else if (s == MachineState::PAUSED) {
    if (!motion_blocked()) {
      fault_clear(Fault::SENSOR_LIMIT);
      set_state(MachineState::RUNNING);
      LOGLN("Resumed");
    }
  }
}

void on_stop(MachineState s) {
  if (s == MachineState::RUNNING) {
    set_state(MachineState::PAUSED);
    LOGLN("Paused");
  } else if (s == MachineState::HOMING) {
    set_state(MachineState::STOPPED);
    fault_set(Fault::HOME_REQUIRED);
    LOGLN("Homing stopped, press HOME again");
  }
}

void on_home(MachineState s) {
  if (s == MachineState::RUNNING) {
    LOGLN("Press STOP before homing");
  } else if (s == MachineState::STOPPED || s == MachineState::PAUSED) {
    if (fault_active(Fault::UNEXPECTED_RESET)) {
      LOGLN("Position was lost, move axes home by hand and press START");
    } else if (!motion_blocked()) {
      set_state(MachineState::HOMING);
      LOGLN("Homing");
    }
  }
}

// Counts detections while running, pauses on the Nth one
void on_sensor(MachineState s) {
  if (s != MachineState::RUNNING) {
    return;
  }

  ++g_sensor_count;
  const unsigned long count = g_sensor_count;

  // Pause first, print after, so serial never delays the stop
  if (g_sensor_count >= SENSOR_TRIGGER_COUNT) {
    g_sensor_count = 0;
    set_state(MachineState::PAUSED);
    fault_set(Fault::SENSOR_LIMIT);
    LOGLN("Sensor limit reached, paused. Press START to resume");
    return;
  }
  LOG("Sensor count %lu/%lu\n", count, static_cast<unsigned long>(SENSOR_TRIGGER_COUNT));
}

}  // namespace

void machine_init() {
  if (g_lock == nullptr) {
    g_lock = xSemaphoreCreateMutex();
  }
  xSemaphoreTake(g_lock, portMAX_DELAY);
  set_state(MachineState::STOPPED);
  xSemaphoreGive(g_lock);
}

void machine_on_input(InputId id, bool active) {
  // Buttons act on press, sensor on detection
  if (!active) {
    return;
  }

  xSemaphoreTake(g_lock, portMAX_DELAY);
  const MachineState s = g_state.load();
  switch (id) {
    case InputId::START:
      on_start(s);
      break;
    case InputId::STOP:
      on_stop(s);
      break;
    case InputId::HOME:
      on_home(s);
      break;
    case InputId::SENSOR:
      on_sensor(s);
      break;
  }
  xSemaphoreGive(g_lock);
}

MachineState machine_state() {
  return g_state.load();
}

void machine_set_activity(Activity activity) {
  g_activity = activity;
}

void machine_homing_finished(bool completed) {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  if (completed && g_state.load() == MachineState::HOMING) {
    fault_clear(Fault::HOME_REQUIRED);
    set_state(MachineState::STOPPED);
    LOGLN("Home done");
  }
  xSemaphoreGive(g_lock);
}

void machine_fault_stop() {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  if (g_state.load() != MachineState::STOPPED) {
    // A stopped cycle leaves the axes away from 0
    set_state(MachineState::STOPPED);
    fault_set(Fault::HOME_REQUIRED);
    LOGLN("Machine stopped on fault");
  }
  xSemaphoreGive(g_lock);
}

// -------------------- run_control --------------------

bool run_halted() {
  if (fault_active(Fault::IO_EXPANDER)) {
    return true;
  }
  const MachineState s = g_state.load();
  switch (g_activity) {
    case Activity::CYCLE:
      return s != MachineState::RUNNING;
    case Activity::HOMING:
      return s != MachineState::HOMING;
    default:
      return true;
  }
}

bool run_wait_resume() {
  for (;;) {
    if (fault_active(Fault::IO_EXPANDER)) {
      return false;
    }
    const MachineState s = g_state.load();
    if (g_activity == Activity::CYCLE) {
      if (s == MachineState::RUNNING) {
        return true;
      }
      if (s != MachineState::PAUSED) {
        return false;  // HOME pressed while paused, drop the cycle
      }
    } else if (g_activity == Activity::HOMING) {
      return s == MachineState::HOMING;
    } else {
      return false;
    }
    delay(5);
  }
}
