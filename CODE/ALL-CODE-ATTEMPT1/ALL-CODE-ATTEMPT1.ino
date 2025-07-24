#include <Wire.h>
#include <math.h>
#include <AccelStepper.h>

// --- Addresses ---
#define HMC5883L_ADDR 0x1E
#define MPU6050_ADDR 0x68
#define DECLINATION_ANGLE 0.6167

// --- Stepper Pins ---
#define DIR_PIN1 5
#define STEP_PIN1 6
#define DIR_PIN2 7
#define STEP_PIN2 8
#define DIR_PIN3 2
#define STEP_PIN3 3
#define ENABLE_PIN3 4

// --- Stepper Settings ---
#define STEPS_PER_REV_DM556 3200
#define STEPS_PER_REV_TMC2208 1600

// --- Gear Ratios ---
#define GEAR_INPUT_1 13
#define GEAR_OUTPUT_1 79
#define GEAR_INPUT_2 22
#define GEAR_OUTPUT_2 120
#define GEAR_INPUT_3 16
#define GEAR_OUTPUT_3 102

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN1, DIR_PIN1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN2, DIR_PIN2);
AccelStepper stepper3(AccelStepper::DRIVER, STEP_PIN3, DIR_PIN3);

// --- Kalman Filter Struct ---
typedef struct {
  float Q_angle;
  float Q_bias;
  float R_measure;
  float angle;
  float bias;
  float rate;
  float P[2][2];
} Kalman;

Kalman kalmanRoll, kalmanPitch;

// --- Sensor Values ---
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AngleRoll, AnglePitch;
unsigned long lastTime;
float dt;

int16_t x, y, z;

// --- Stepper Position Tracking ---
float currentHeading = 0;
float currentElevation = 0;

// --- State ---
bool isRunning = false;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  pinMode(ENABLE_PIN3, OUTPUT);
  digitalWrite(ENABLE_PIN3, HIGH);

  stepper1.setMaxSpeed(1000);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00);
  Wire.write(0x18);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01);
  Wire.write(0xA0);
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  kalmanRoll = {0.001, 0.003, 0.03, 0, 0, 0, {{0, 0}, {0, 0}}};
  kalmanPitch = kalmanRoll;
  lastTime = micros();

  Serial.println("System ready.");
}

void loop() {
  readMPU6050();

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float gyroXrate = GyroX / 65.5;
  float gyroYrate = GyroY / 65.5;

  float accRoll = atan2(AccY, AccZ) * 180.0 / PI;
  float accPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  AngleRoll = kalmanUpdate(&kalmanRoll, accRoll, gyroXrate, dt);
  AnglePitch = kalmanUpdate(&kalmanPitch, accPitch, gyroYrate, dt);

  readHMC5883L();

  float heading = calculateHeadingYForward(x, y);
  heading += DECLINATION_ANGLE;
  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;

  Serial.print("Heading: "); Serial.print(heading);
  Serial.print(" Elevation: "); Serial.println(AngleRoll);

  handleSerial();

  stepper1.run();
  stepper2.run();
  stepper3.run();
}

float kalmanUpdate(Kalman* kf, float newAngle, float newRate, float dt) {
  kf->rate = newRate - kf->bias;
  kf->angle += dt * kf->rate;
  kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;
  float y = newAngle - kf->angle;
  float S = kf->P[0][0] + kf->R_measure;
  float K0 = kf->P[0][0] / S;
  float K1 = kf->P[1][0] / S;
  kf->angle += K0 * y;
  kf->bias += K1 * y;
  float P00_temp = kf->P[0][0];
  float P01_temp = kf->P[0][1];
  kf->P[0][0] -= K0 * P00_temp;
  kf->P[0][1] -= K0 * P01_temp;
  kf->P[1][0] -= K1 * P00_temp;
  kf->P[1][1] -= K1 * P01_temp;
  return kf->angle;
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 14);

  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  AccX = (float)rawAccX / 4096.0 - 0.07;
  AccY = (float)rawAccY / 4096.0 + 0.01;
  AccZ = (float)rawAccZ / 4096.0 + 0.03;
  GyroX = (float)rawGyroX;
  GyroY = (float)rawGyroY;
  GyroZ = (float)rawGyroZ;
}

void readHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);
  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }
}

float calculateHeadingYForward(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x) * (180.0 / PI);
  heading -= 180.0;
  if (heading < 0) heading += 360.0;
  return heading;
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

void moveToTarget(int motor, float targetDegree, float& currentDegree) {
  float ratio = 1.0;
  int stepsPerRev = 0;
  switch (motor) {
    case 1: ratio = (float)GEAR_OUTPUT_1 / GEAR_INPUT_1; stepsPerRev = STEPS_PER_REV_DM556; break;
    case 2: ratio = (float)GEAR_OUTPUT_2 / GEAR_INPUT_2; stepsPerRev = STEPS_PER_REV_DM556; break;
    case 3: ratio = (float)GEAR_OUTPUT_3 / GEAR_INPUT_3; stepsPerRev = STEPS_PER_REV_TMC2208; digitalWrite(ENABLE_PIN3, LOW); break;
  }
  float delta = targetDegree - currentDegree;
  long steps = (delta / 360.0) * ratio * stepsPerRev;
  currentDegree = targetDegree;
  switch (motor) {
    case 1: stepper1.move(steps); break;
    case 2: stepper2.move(steps); break;
    case 3: stepper3.move(steps); break;
  }
}

void parseMoveCommand(String cmd) {
  cmd.trim();
  cmd.replace("  ", " ");
  int firstSpace = cmd.indexOf(' ');
  int secondSpace = cmd.indexOf(' ', firstSpace + 1);
  if (firstSpace == -1 || secondSpace == -1) return;
  int motor = cmd.substring(firstSpace + 1, secondSpace).toInt();
  float degrees = cmd.substring(secondSpace + 1).toFloat();
  if (motor < 1 || motor > 3) return;
  moveDegrees(motor, degrees);
}

void handleSerial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.startsWith("move")) {
    parseMoveCommand(input);
  } else {
    int separator = input.indexOf(',');
    if (separator != -1) {
      float targetHeading = input.substring(0, separator).toFloat();
      float targetElevation = input.substring(separator + 1).toFloat();
      moveToTarget(1, targetHeading, currentHeading);
      moveToTarget(2, targetElevation, currentElevation);
    }
  }
}