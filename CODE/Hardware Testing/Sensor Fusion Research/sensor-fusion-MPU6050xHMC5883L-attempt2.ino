#include <Wire.h>
#include <math.h>

// --- Addresses ---
#define HMC5883L_ADDR 0x1E
#define MPU6050_ADDR 0x68
#define DECLINATION_ANGLE 0.6167  // degrees

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

// --- Magnetometer ---
int16_t x, y, z;

// --- Kalman Update Function ---
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

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // --- Init HMC5883L ---
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00);
  Wire.write(0x18); // 1-average, 75 Hz
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01);
  Wire.write(0xA0); // Gain
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02);
  Wire.write(0x00); // Continuous
  Wire.endTransmission();

  // --- Init MPU6050 ---
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00); // ±2g
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);
  Wire.write(0x08); // ±500°/s
  Wire.endTransmission();

  // Kalman Filter Init
  kalmanRoll = {0.001, 0.003, 0.03, 0, 0, 0, {{0, 0}, {0, 0}}};
  kalmanPitch = kalmanRoll;

  lastTime = micros();
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

  Serial.print(heading);
  Serial.print(",");
  Serial.println(AngleRoll);

  delay(20);
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
  heading -= 180.0;  // custom mounting shift
  if (heading < 0) heading += 360.0;
  return heading;
}
