#include <Arduino.h>
#include <esp_system.h>

#include "config.h"
#include "dc_motor.h"
#include "fault.h"
#include "indicator.h"
#include "logger.h"
#include "inputs.h"
#include "io_expander.h"
#include "machine.h"
#include "sequence.h"
#include "solenoid.h"
#include "stepper_motor.h"

namespace {

// No expander means no driver enables, so the machine must not run.
// Keep retrying until it answers, RED shows the fault meanwhile.
void wait_for_expander() {
  while (!io_expander_init()) {
    fault_set(Fault::IO_EXPANDER);
    LOG("IO expander not found at 0x%02X, check I2C wiring\n", IO_EXPANDER_ADDR);
    delay(1000);
  }
  fault_clear(Fault::IO_EXPANDER);
}

// A crash or power dip mid-cycle leaves the axes somewhere unknown
void check_reset_reason() {
  switch (esp_reset_reason()) {
    case ESP_RST_BROWNOUT:
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      fault_set(Fault::UNEXPECTED_RESET);
      LOGLN("Check the axes are at home, then press START to confirm");
      break;
    default:
      break;
  }
}

}  // namespace

void setup() {
  LOG_BEGIN(115200);

  indicator_init();
  check_reset_reason();
  wait_for_expander();

  solenoid_set(false);
  dc_motor_init();
  stepper_init();

  machine_init();
  inputs_init(machine_on_input);

  LOGLN("Ready. Axes taken as home. Press START.");
}

void loop() {
  // Expander lost: motion already halted by the run gate, make it final
  if (fault_active(Fault::IO_EXPANDER) && machine_state() != MachineState::STOPPED) {
    dc_stop_all();
    machine_fault_stop();
  }

  switch (machine_state()) {
    case MachineState::RUNNING:
      machine_set_activity(Activity::CYCLE);
      sequence_run_cycle();
      machine_set_activity(Activity::NONE);
      break;

    case MachineState::HOMING: {
      machine_set_activity(Activity::HOMING);
      const bool done = sequence_home();
      machine_set_activity(Activity::NONE);
      machine_homing_finished(done);
      break;
    }

    default:
      delay(10);
      break;
  }
}
