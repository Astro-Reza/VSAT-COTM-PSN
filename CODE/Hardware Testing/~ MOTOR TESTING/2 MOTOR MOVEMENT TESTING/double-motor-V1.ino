#include <AccelStepper.h>

// --- Pin Definitions ---
#define STEP_PIN 5
#define DIR_PIN 6

// --- Stepper Settings ---
#define STEPS_PER_REV 3200  // For example, DM556 with 1/8 microstepping

// --- Gear Ratio (Output : Input) ---
#define GEAR_INPUT 13
#define GEAR_OUTPUT 79

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  Serial.begin(115200);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);

  Serial.println("Ready. Use: move <degrees>");
}

void loop() {
  handleSerial();
  stepper.run();
}

// --- Serial Input Parser ---
void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.startsWith("move")) {
    input.replace("  ", " ");
    int spaceIdx = input.indexOf(' ');
    if (spaceIdx == -1) {
      Serial.println("Invalid format. Use: move <degrees>");
      return;
    }

    float degrees = input.substring(spaceIdx + 1).toFloat();
    float ratio = (float)GEAR_OUTPUT / GEAR_INPUT;
    long steps = (degrees / 360.0) * ratio * STEPS_PER_REV;

    Serial.print("Moving ");
    Serial.print(degrees);
    Serial.print(" degrees (");
    Serial.print(steps);
    Serial.println(" steps)");

    stepper.move(steps);
  }
}
