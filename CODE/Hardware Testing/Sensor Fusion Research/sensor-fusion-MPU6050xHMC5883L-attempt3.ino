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

// --- Complementary Filter ---
float fusedRoll, fusedPitch, fusedHeading;
float alpha = 0.98; // Complementary filter coefficient (gyro weight)

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

  // Initialize fused angles
  fusedRoll = 0;
  fusedPitch = 0;
  fusedHeading = 0;

  lastTime = micros();
}

void loop() {
  readMPU6050();
  readHMC5883L();

  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Calculate roll and pitch from accelerometer
  float accRoll = atan2(AccY, AccZ) * 180.0 / PI;
  float accPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // Kalman filter for roll and pitch
  float gyroXrate = GyroX / 65.5;
  float gyroYrate = GyroY / 65.5;
  AngleRoll = kalmanUpdate(&kalmanRoll, accRoll, gyroXrate, dt);
  AnglePitch = kalmanUpdate(&kalmanPitch, accPitch, gyroYrate, dt);

  // Complementary filter for roll and pitch
  fusedRoll = alpha * (fusedRoll + gyroXrate * dt) + (1 - alpha) * accRoll;
  fusedPitch = alpha * (fusedPitch + gyroYrate * dt) + (1 - alpha) * accPitch;

  // Tilt-compensated heading
  float rollRad = fusedRoll * PI / 180.0;
  float pitchRad = fusedPitch * PI / 180.0;

  // Compensate magnetometer readings for tilt
  float xh = x * cos(pitchRad) + z * sin(pitchRad);
  float yh = y * cos(rollRad) - z * sin(rollRad);

  // Calculate heading
  float heading = atan2(yh, xh) * 180.0 / PI;
  heading += DECLINATION_ANGLE;
  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;

  // Complementary filter for heading (optional, using gyroZ for drift correction)
  float gyroZrate = GyroZ / 65.5;
  fusedHeading = alpha * (fusedHeading + gyroZrate * dt) + (1 - alpha) * heading;

  // Output fused angles
  Serial.print(fusedHeading);
  Serial.print(",");
  Serial.print(fusedRoll);
  Serial.print(",");
  Serial.println(fusedPitch);

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
  Wire.read(); Wire.read(); // Skip temperature
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