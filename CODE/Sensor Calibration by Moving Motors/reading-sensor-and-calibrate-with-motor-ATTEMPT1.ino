#include <AccelStepper.h>
#include <Wire.h>

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

// MPU6050 I2C address
#define MPU6050_ADDR 0x68
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B

// HMC5883L I2C address
#define HMC5883L_ADDR 0x1E
#define ACCEL_SCALE 16384.0
#define MAG_SCALE 0.92

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);

double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0;
bool isCalibrating = false;

bool isRunning = false;
int sequenceStep = 0;
bool waitingForNext = false;

void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN3, OUTPUT);
  digitalWrite(ENABLE_PIN3, HIGH);

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  MPU6050_init();
  HMC5883L_init();

  Serial.println("Stepper system ready.");
  Serial.println("Commands:\n  startseq\n  stopseq\n  move x y\n  calibrate_motor");
}

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
      digitalWrite(ENABLE_PIN3, LOW);
      break;
  }

  long steps = (degrees / 360.0) * ratio * stepsPerRev;

  switch (motor) {
    case 1: stepper1.move(steps); break;
    case 2: stepper2.move(steps); break;
    case 3: stepper3.move(steps); break;
  }
}

void runSequenceStep(int step) {
  switch (step) {
    case 0:
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 1:
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    case 2:
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 3:
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    case 4:
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 5:
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    case 6:
      moveDegrees(1, 30);
      moveDegrees(2, 30);
      waitingForNext = true;
      break;
    case 7:
      moveDegrees(1, -30);
      moveDegrees(2, -30);
      waitingForNext = true;
      break;
    default:
      Serial.println("Sequence complete.");
      sequenceStep = 0;
      isRunning = false;
      digitalWrite(ENABLE_PIN3, HIGH);
      return;
  }
  sequenceStep++;
}

void calibrate_HMC5883L() {
  Serial.println("Calibrating HMC5883L with motor movement...");
  double mx, my, mz;
  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  digitalWrite(ENABLE_PIN3, LOW);

  for (int phase = 0; phase < 4; phase++) {
    switch (phase) {
      case 0:
        moveDegrees(1, 180);
        moveDegrees(2, -90);
        moveDegrees(3, 90);
        break;
      case 1:
        moveDegrees(1, -180);
        moveDegrees(2, 180);
        moveDegrees(3, -90);
        break;
      case 2:
        moveDegrees(1, 90);
        moveDegrees(2, -90);
        moveDegrees(3, 90);
        break;
      case 3:
        moveDegrees(1, -90);
        moveDegrees(2, 0);
        moveDegrees(3, -90);
        break;
    }

    while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
      stepper1.run();
      stepper2.run();
      stepper3.run();

      read_HMC5883L(mx, my, mz);
      if (mx < magMinX) magMinX = mx; if (mx > magMaxX) magMaxX = mx;
      if (my < magMinY) magMinY = my; if (my > magMaxY) magMaxY = my;
      if (mz < magMinZ) magMinZ = mz; if (mz > magMaxZ) magMaxZ = mz;
    }
  }

  digitalWrite(ENABLE_PIN3, HIGH);
  Serial.println("Calibration with movement complete.");
}

void handleSerial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input == "startseq") {
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    digitalWrite(ENABLE_PIN3, LOW);
    sequenceStep = 0;
    isRunning = true;
    waitingForNext = false;
    runSequenceStep(sequenceStep);

  } else if (input == "stopseq") {
    isRunning = false;
    stepper1.stop();
    stepper2.stop();
    stepper3.stop();
    digitalWrite(ENABLE_PIN3, HIGH);
    Serial.println("Sequence stopped.");

  } else if (input.startsWith("move")) {
    parseMoveCommand(input);

  } else if (input == "calibrate_motor") {
    Serial.println("Starting magnetometer calibration with demo movements...");
    isCalibrating = true;
  }
}

void parseMoveCommand(String cmd) {
  cmd.trim();
  cmd.replace("  ", " ");
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

  moveDegrees(motor, degrees);
}

void read_HMC5883L(double &mx, double &my, double &mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);

  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();

  mx = (x - (magMinX + magMaxX) / 2) * MAG_SCALE;
  my = (y - (magMinY + magMaxY) / 2) * MAG_SCALE;
  mz = (z - (magMinZ + magMaxZ) / 2) * MAG_SCALE;
}

void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0);
  Wire.endTransmission(true);
}

void HMC5883L_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); Wire.write(0x70); Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); Wire.write(0x20); Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
}

void loop() {
  handleSerial();

  if (isCalibrating) {
    calibrate_HMC5883L();
    isCalibrating = false;
  }

  stepper1.run();
  stepper2.run();
  stepper3.run();

  if (isRunning && waitingForNext &&
      !stepper1.isRunning() && !stepper2.isRunning() && !stepper3.isRunning()) {
    delay(100);
    runSequenceStep(sequenceStep);
    waitingForNext = false;
  }
}
