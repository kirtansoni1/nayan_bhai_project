// Simple stepper test, plain AccelStepper. Runs on its own, no machine firmware.
//   pio run -e stepper_test -t upload
//   pio device monitor

#include <AccelStepper.h>
#include <Arduino.h>
#include <TCA9555.h>
#include <Wire.h>

// -------- Settings --------
#define STEP_PIN    40      // S_M1 = 1,  S_M2 = 42, S_M3 = 40
#define DIR_PIN     39      // S_M1 = 2,  S_M2 = 41, S_M3 = 39
#define STEPS       3200    // steps per move
#define MAX_SPEED   12000   // steps/s
#define ACCEL       8000    // steps/s^2
#define CYCLES      20      // CW + CCW pairs
#define DWELL_MS    500     // wait after each move

// -------- Expander (enables stepper drivers) --------
#define EXPANDER_ADDR 0x20
#define PORT0_OUT     0x00  // P5-P7 low = steppers enabled, P1-P4 low = solenoid and DC off
#define PORT0_DIR     0x01  // P1-P7 outputs, P0 input

AccelStepper motor(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
TCA9535 expander(EXPANDER_ADDR);

void move(long steps) {
  long start = motor.currentPosition();
  uint32_t t = millis();

  motor.move(steps);
  while (motor.run()) {
  }

  Serial.printf("%s  steps %ld  pos %ld  time %lu ms\n", steps > 0 ? "CW " : "CCW",
                motor.currentPosition() - start, motor.currentPosition(), millis() - t);
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  Wire.begin(10, 11);
  if (!expander.isConnected()) {
    Serial.println("Expander not found");
  }
  expander.write8(0, PORT0_OUT);
  expander.pinMode8(0, PORT0_DIR);

  motor.setMaxSpeed(MAX_SPEED);
  motor.setAcceleration(ACCEL);

  Serial.println("Stepper test start");
}

void loop() {
  static int cycle = 0;
  if (cycle >= CYCLES) {
    return;
  }
  cycle++;

  Serial.printf("Cycle %d\n", cycle);
  move(STEPS);
  delay(DWELL_MS);
  move(-STEPS);
  delay(DWELL_MS);

  if (cycle == CYCLES) {
    Serial.printf("Done. Final pos %ld\n", motor.currentPosition());
  }
}
