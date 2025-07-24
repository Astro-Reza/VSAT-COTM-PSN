#include <Wire.h>
#include <AccelStepper.h>

// --- Pin Definitions FOR GEARS ---
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
bool isCalibrating = false;

// MPU6050 I2C address
#define MPU6050_ADDR 0x68
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1   0x6B

// HMC5883L I2C address
#define HMC5883L_ADDR 0x1E

#define ACCEL_SCALE 16384.0
#define MAG_SCALE 0.92

double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0;

void setup() {
  Serial.begin(115200); 
  Wire.begin();

  pinMode(ENABLE_PIN3, OUTPUT);
  digitalWrite(ENABLE_PIN3, HIGH);  // Disable TMC2208 initially (active-low)

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  MPU6050_init();
  HMC5883L_init();

  calibrate_MPU6050();
  calibrate_HMC5883L();

  Serial.println("Stepper system ready.");
  Serial.println("Commands:");
  Serial.println("  startseq     - Start automatic sequence");
  Serial.println("  stopseq      - Stop sequence");
  Serial.println("  move x y     - Move motor x by y degrees");
}

// -------- sensory system --------
void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1);
  Wire.write(0); // Wake up MPU6050
  Wire.endTransmission(true);
}

void calibrate_MPU6050() {
  double ax, ay, az;
  int samples = 100;

  for (int i = 0; i < samples; i++) {
    read_MPU6050(ax, ay, az);
    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;
    delay(10);
  }

  accelOffsetX /= samples;
  accelOffsetY /= samples;
  accelOffsetZ = (accelOffsetZ / samples) - 1.0; // adjust for gravity
}

void read_MPU6050(double &ax, double &ay, double &az) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  ax = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetX;
  ay = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetY;
  az = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetZ;
}

void HMC5883L_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); Wire.write(0x70); Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); Wire.write(0x20); Wire.endTransmission();
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
}

void calibrate_HMC5883L() {
  isCalibrating = true;
  Serial.println("Calibrating HMC5883L with predefined movements...");
  double mx, my, mz;
  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  digitalWrite(ENABLE_PIN3, LOW);

  for (int step = 0; step < 8; step++) {
    Serial.print("Step ");
    Serial.println(step + 1);

    switch (step) {
      case 0: moveDegrees(1, 30); moveDegrees(2, 30); moveDegrees(3, 30); break;
      case 1: moveDegrees(1, -30); moveDegrees(2, -30); moveDegrees(3, -30); break;
      case 2: moveDegrees(1, 30); moveDegrees(2, 30); moveDegrees(3, 30); break;
      case 3: moveDegrees(1, -30); moveDegrees(2, -30); moveDegrees(3, -30); break;
      case 4: moveDegrees(1, 30); moveDegrees(2, 30); moveDegrees(3, 30); break;
      case 5: moveDegrees(1, -30); moveDegrees(2, -30); moveDegrees(3, -30); break;
      case 6: moveDegrees(1, 30); moveDegrees(2, 30); moveDegrees(3, 30); break;
      case 7: moveDegrees(1, -30); moveDegrees(2, -30); moveDegrees(3, -30); break;
    }

    // Run until both motors reach the target
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0) {
      stepper1.run();
      stepper2.run();
    }

    // Now collect magnetic data for a short, stable window
    unsigned long tStart = millis();
    while (millis() - tStart < 3000) {
      read_HMC5883L(mx, my, mz);
      if (mx < magMinX) magMinX = mx; if (mx > magMaxX) magMaxX = mx;
      if (my < magMinY) magMinY = my; if (my > magMaxY) magMaxY = my;
      if (mz < magMinZ) magMinZ = mz; if (mz > magMaxZ) magMaxZ = mz;
      delay(100);
    }
  }
  isCalibrating = false;
  digitalWrite(ENABLE_PIN3, HIGH);
  Serial.println("Calibration done.");
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

// --- Serial Commands ---
void handleSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.startsWith("move")) {
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
  }

  if (!isCalibrating && isRunning) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }

  double ax, ay, az;
  double mx, my, mz;
  double pitch, roll, yaw;

  read_MPU6050(ax, ay, az);
  pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  roll  = atan2(ay, az) * 180.0 / PI;

  read_HMC5883L(mx, my, mz);
  yaw = atan2(my, mx);

  // Adjust yaw with magnetic declination (example: -10°13' = -0.1783 rad)
  float declinationAngle = -0.1783;
  yaw += declinationAngle;

  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  yaw *= 180.0 / PI;

  Serial.print(pitch);
  Serial.print(",");
  Serial.print(roll);
  Serial.print(",");
  Serial.println(yaw);

  delay(200);
}
