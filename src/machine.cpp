#include "machine.h"

#include <atomic>

#include "config.h"
#include "fault.h"
#include "indicator.h"
#include "logger.h"
#include "run_control.h"
#include "sensor_counter.h"

namespace {

std::atomic<MachineState> g_state{MachineState::STOPPED};
SemaphoreHandle_t g_lock = nullptr;

// Main task only
Activity g_activity = Activity::NONE;

// Guarded by g_lock
SensorCounter g_sensor(SENSOR_TRIGGER_COUNT);

// Caller holds g_lock
void set_state(MachineState s) {
  g_state.store(s);
  const bool moving = (s == MachineState::RUNNING || s == MachineState::JOGGING);
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

// Caller holds g_lock
void log_sensor_hit() {
  const unsigned long count = g_sensor.count();
  const unsigned long limit = g_sensor.limit();
  if (g_sensor.limit_reached()) {
    LOG("Sensor count %lu/%lu, stopping after this cycle\n", count, limit);
  } else {
    LOG("Sensor count %lu/%lu\n", count, limit);
  }
}

void on_start(MachineState s) {
  if (s == MachineState::STOPPED) {
    if (fault_active(Fault::UNEXPECTED_RESET)) {
      // First press only acknowledges, operator must have checked the axes
      fault_clear(Fault::UNEXPECTED_RESET);
      LOGLN("Restart acknowledged, press START again to run");
    } else if (motion_blocked()) {
      // reason already printed
    } else if (input_is_active(InputId::SENSOR)) {
      fault_set(Fault::SENSOR_BLOCKED);
      LOGLN("Sensor blocked, clear it and press START");
    } else {
      fault_clear(Fault::SENSOR_LIMIT);
      fault_clear(Fault::SENSOR_BLOCKED);
      g_sensor.reset();
      set_state(MachineState::RUNNING);
      LOGLN("Starting cycle");
    }
  } else if (s == MachineState::PAUSED) {
    if (!motion_blocked()) {
      set_state(MachineState::RUNNING);
      LOGLN("Resumed");
    }
  }
}

void on_stop(MachineState s) {
  if (s == MachineState::RUNNING) {
    set_state(MachineState::PAUSED);
    LOGLN("Paused");
  } else if (s == MachineState::JOGGING) {
    set_state(MachineState::STOPPED);
    LOGLN("Jog stopped");
  }
}

// HOME jogs S_M3, only from STOPPED
void on_home(MachineState s) {
  if (s == MachineState::RUNNING || s == MachineState::PAUSED) {
    LOGLN("HOME works only when stopped");
  } else if (s == MachineState::STOPPED) {
    if (fault_active(Fault::UNEXPECTED_RESET)) {
      LOGLN("Press START to acknowledge the restart first");
    } else if (!motion_blocked()) {
      set_state(MachineState::JOGGING);
      LOGLN("Jogging S_M3");
    }
  }
}

// Detections count only inside the window, also while paused there
void on_sensor(MachineState s) {
  if (s != MachineState::RUNNING && s != MachineState::PAUSED) {
    return;
  }
  if (g_sensor.detect()) {
    log_sensor_hit();
  }
}

}  // namespace

void machine_init() {
  if (g_lock == nullptr) {
    g_lock = xSemaphoreCreateMutex();
  }
  xSemaphoreTake(g_lock, portMAX_DELAY);
  g_sensor.reset();
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

void machine_cycle_finished(bool completed) {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  const bool was_open = g_sensor.window_open();
  const bool hit = g_sensor.hit();
  const uint32_t count = g_sensor.count();
  const bool stop = g_sensor.close_window();

  if (completed) {
    if (stop) {
      set_state(MachineState::STOPPED);
      fault_set(Fault::SENSOR_LIMIT);
      LOGLN("Sensor limit reached, stopped. Press START to run again");
    } else if (was_open && !hit && count > 0) {
      LOGLN("No detection this cycle, sensor count reset");
    }
  }
  xSemaphoreGive(g_lock);
}

void machine_jog_finished() {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  if (g_state.load() == MachineState::JOGGING) {
    set_state(MachineState::STOPPED);
  }
  xSemaphoreGive(g_lock);
}

void machine_fault_stop() {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  g_sensor.reset();
  if (g_state.load() != MachineState::STOPPED) {
    set_state(MachineState::STOPPED);
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
    case Activity::JOG:
      return s != MachineState::JOGGING;
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
        return false;  // stopped on fault, drop the cycle
      }
    } else if (g_activity == Activity::JOG) {
      return s == MachineState::JOGGING;
    } else {
      return false;
    }
    delay(5);
  }
}

void run_sensor_window_open() {
  xSemaphoreTake(g_lock, portMAX_DELAY);
  // Stable level is updated before its edge is handled, so no detection is lost or doubled
  g_sensor.open_window(input_is_active(InputId::SENSOR));
  if (g_sensor.hit()) {
    log_sensor_hit();
  }
  xSemaphoreGive(g_lock);
}
