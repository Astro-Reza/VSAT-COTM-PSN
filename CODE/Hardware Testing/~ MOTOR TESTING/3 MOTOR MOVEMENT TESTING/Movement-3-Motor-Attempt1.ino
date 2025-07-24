#include <AccelStepper.h>

// --- Pin Definitions ---
#define DIR_PIN1 8
#define STEP_PIN1 9
#define DIR_PIN2 6
#define STEP_PIN2 7
#define DIR_PIN3 3
#define STEP_PIN3 4
#define ENABLE_PIN3 5  // Only for TMC2208
                                                                                                     
// --- Stepper Settings ---
#define STEPS_PER_REV_DM556 3200     // 1/8 microstep
#define STEPS_PER_REV_TMC2208 1600   // 1/8 microstep

// --- Gear Ratios (Output : Input) ---
#define GEAR_INPUT_1 13
#define GEAR_OUTPUT_1 79

#define GEAR_INPUT_2 22
#define GEAR_OUTPUT_2 120


#define GEAR_INPUT_3 20
#define GEAR_OUTPUT_3 100

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);

// --- State ---
bool isRunning = false;
int sequenceStep = 0;
bool waitingForNext = false;

// --- Setup ---
void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN3, OUTPUT);
  digitalWrite(ENABLE_PIN3, HIGH);  // Disable TMC2208 initially (active-low)

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  Serial.println("Stepper system ready.");
  Serial.println("Commands:");
  Serial.println("  startseq     - Start automatic sequence");
  Serial.println("  stopseq      - Stop sequence");
  Serial.println("  move x y     - Move motor x by y degrees");
}

// --- Movement Helper ---
void moveDegrees(int motor, float degrees) {
  float ratio = 1.0;
  int stepsPerRev = 0;

  switch (motor) {
    case 1:
      ratio = (float)GEAR_OUTPUT_1 / GEAR_INPUT_1;
      stepsPerRev = STEPS_PER_REV_DM556;
      break;
    case 2:
      ratio = (float)GEAR_OUTPUT_2 / GEAR_INPUT_2;
      stepsPerRev = STEPS_PER_REV_DM556;
      break;
    case 3:
      ratio = (float)GEAR_OUTPUT_3 / GEAR_INPUT_3;
      stepsPerRev = STEPS_PER_REV_TMC2208;
      digitalWrite(ENABLE_PIN3, LOW);  // Enable TMC2208
      break;
  }

  long steps = (degrees / 360.0) * ratio * stepsPerRev;

  switch (motor) {
    case 1: stepper1.move(steps); break;
    case 2: stepper2.move(steps); break;
    case 3: stepper3.move(steps); break;
  }
}

// --- Sequence Logic ---
void runSequenceStep(int step) {
  switch (step) {
    case 0:
      Serial.println("Move 1");
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 1:
      Serial.println("Move 2");
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    case 2:
      Serial.println("Move 3");
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 3:
      Serial.println("Move 4");
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    case 4:
      Serial.println("Reverse 1");
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 5:
      Serial.println("Reverse 2 - Undo polarisasi");
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    case 6:
      Serial.println("Reverse 3");
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 7:
      Serial.println("Reverse 4");
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    default:
      Serial.println("Sequence complete. Restarting...");
      sequenceStep = 0;
      isRunning = false;
      digitalWrite(20, HIGH);  // Disable TMC2208
      return;
  }

  sequenceStep++;
}

// --- Serial Commands ---
void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "startseq") {
    Serial.println("Starting sequence...");
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    digitalWrite(ENABLE_PIN3, LOW);  // Enable TMC2208
    sequenceStep = 0;
    isRunning = true;
    waitingForNext = false;
    runSequenceStep(sequenceStep);
  } else if (input == "stopseq") {
    isRunning = false;
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    digitalWrite(ENABLE_PIN3, HIGH);  // Disable TMC2208
    Serial.println("Sequence stopped.");
  } else if (input.startsWith("move")) {
    parseMoveCommand(input);
  }
}

// --- Parse Manual Move Command ---
void parseMoveCommand(String cmd) {
  cmd.trim();
  cmd.replace("  ", " ");  // Replace double spaces
  int firstSpace = cmd.indexOf(' ');
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);

  if (firstSpace == -1 || secondSpace == -1) {
    Serial.println("Invalid format. Use: move <motor> <degrees>");
    return;
  }

  int motor = cmd.substring(firstSpace + 1, secondSpace).toInt();
  float degrees = cmd.substring(secondSpace + 1).toFloat();

  if (motor < 1 || motor > 3) {
    Serial.println("Invalid motor number. Must be 1, 2, or 3.");
    return;
  }

  Serial.print("Moving motor ");
  Serial.print(motor);
  Serial.print(" by ");
  Serial.print(degrees);
  Serial.println(" degrees");

  moveDegrees(motor, degrees);
}

// --- Main Loop ---
void loop() {
  handleSerial();

  if (isRunning) {
    stepper1.run();
    stepper2.run();
    stepper3.run();

    if (waitingForNext && !stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning()) {
      delay(100);
      runSequenceStep(sequenceStep);
      waitingForNext = false;
    }
  } else {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}
