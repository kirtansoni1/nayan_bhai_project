#include "sequence.h"

#include <Arduino.h>

#include "config.h"
#include "dc_motor.h"
#include "logger.h"
#include "run_control.h"
#include "solenoid.h"
#include "stepper_motor.h"

namespace {

// Step 7: stepper 3 CW 3 inch and stepper 2 CCW 1 inch together
const StepperMove kStep7[] = {
  {3, S_M3_STROKE_STEPS, Direction::CW},
  {2, S_M2_STROKE_STEPS, Direction::CCW},
};
constexpr uint8_t kStep7Count = sizeof(kStep7) / sizeof(kStep7[0]);

// Delay that freezes with the machine and continues the leftover time on resume
bool wait_ms(uint32_t ms) {
  uint32_t left = ms;
  uint32_t since = millis();

  for (;;) {
    const uint32_t elapsed = millis() - since;
    if (elapsed >= left) {
      return true;
    }
    if (run_halted()) {
      left -= elapsed;
      if (!run_wait_resume()) {
        return false;
      }
      since = millis();
    }
    delay(1);
  }
}

// Solenoid switch that respects pause, so it never toggles while stopped
bool set_solenoid(bool on) {
  if (!run_wait_resume()) {
    return false;
  }
  solenoid_set(on);
  return true;
}

void print_positions(const char *label) {
  LOG("%s S1=%ld S2=%ld S3=%ld\n", label,
      static_cast<long>(stepper_get_position(1)),
      static_cast<long>(stepper_get_position(2)),
      static_cast<long>(stepper_get_position(3)));
}

}  // namespace

// Update this function to edit the machine run sequence.
bool sequence_run_cycle() {
  return
    // 1. DC1 300 RPM CW
    dc_run_ms_blocking(DcMotorId::DC1_300, DC1_300_RUN_MS, DC1_300_SPEED, Direction::CW) &&
    // 2. Stepper 1 CCW 3 inch
    stepper_run_steps_blocking(1, S_M1_STROKE_STEPS, Direction::CCW) &&
    // 3. Stepper 2 CW 1 inch
    stepper_run_steps_blocking(2, S_M2_STROKE_STEPS, Direction::CW) &&
    // 4. Solenoid ON
    set_solenoid(true) &&
    // 5. DC2 300 RPM CW
    dc_run_ms_blocking(DcMotorId::DC2_300, DC2_300_RUN_MS, DC2_300_SPEED, Direction::CW) &&
    // 6. Stepper 1 CW 3 inch
    stepper_run_steps_blocking(1, S_M1_STROKE_STEPS, Direction::CW) &&
    // 7. Stepper 3 CW 3 inch and Stepper 2 CCW 1 inch together
    stepper_run_steps_batch_blocking(kStep7, kStep7Count) &&
    // 8. DC 3000 RPM CW
    dc_run_ms_blocking(DcMotorId::DC_3000, DC_3000_RUN_MS, DC_3000_SPEED, Direction::CW) &&
    // 9. Stepper 3 CCW 3 inch
    stepper_run_steps_blocking(3, S_M3_STROKE_STEPS, Direction::CCW) &&
    wait_ms(800) &&
    // 10. Solenoid OFF and DC2 300 RPM CCW together
    set_solenoid(false) &&
    dc_run_ms_blocking(DcMotorId::DC2_300, DC2_300_RUN_MS, DC2_300_SPEED, Direction::CCW) &&
    // Gap before the loop repeats
    wait_ms(CYCLE_GAP_MS);
}

bool sequence_home() {
  dc_stop_all();
  solenoid_set(false);
  print_positions("Homing from");

  const bool done = stepper_home_all_blocking();
  print_positions(done ? "At home" : "Homing halted at");
  return done;
}
