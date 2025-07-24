#include <Wire.h>
#include <math.h>

// Kalman filter struct
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

// Sensor values
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float AngleRoll, AnglePitch;
unsigned long lastTime;

float dt;

// fungsi kalman
float kalmanUpdate(Kalman* kf, float newAngle, float newRate, float dt) {
  // predict
  kf->rate = newRate - kf->bias;
  kf->angle += dt * kf->rate;

  kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle); // update error covariance matrix
  kf->P[0][1] -= dt * kf->P[1][1];
  kf->P[1][0] -= dt * kf->P[1][1];
  kf->P[1][1] += kf->Q_bias * dt;

  float y = newAngle - kf->angle;
  float S = kf->P[0][0] + kf->R_measure;
  float K0 = kf->P[0][0] / S;
  float K1 = kf->P[1][0] / S;
  kf->angle += K0 * y; // update state
  kf->bias += K1 * y;

  // update matrix
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
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x00);  // 0x00 sets AFS_SEL = 0, which is ±2g
  Wire.endTransmission();

  // Configure gyroscope ±500°/s
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Initialize Kalman filters
  kalmanRoll = {0.001, 0.003, 0.03, 0, 0, 0, {{0, 0}, {0, 0}}};
  kalmanPitch = kalmanRoll;

  lastTime = micros();
}

void loop() {
  readMPU6050();

  // Delta time calculation
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Convert gyro data to deg/s
  float gyroXrate = GyroX / 65.5;
  float gyroYrate = GyroY / 65.5;

  // Accel angle estimation
  float accRoll = atan2(AccY, AccZ) * 180.0 / PI;
  float accPitch = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;

  // Kalman filter fusion
  AngleRoll = kalmanUpdate(&kalmanRoll, accRoll, gyroXrate, dt);
  AnglePitch = kalmanUpdate(&kalmanPitch, accPitch, gyroYrate, dt);

  // Print result
  Serial.print(accRoll);       // Raw accel roll
  Serial.print(",");
  Serial.print(accPitch);      // Raw accel pitch
  Serial.print(",");
  Serial.print(AngleRoll);     // Kalman roll
  Serial.print(",");
  Serial.println(AnglePitch);  // Kalman pitch


  delay(50);
}

void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // skip temperature
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw data to physical units
  AccX = (float)rawAccX / 4096.0 - 0.07;  // ±8g
  AccY = (float)rawAccY / 4096.0 + 0.01;
  AccZ = (float)rawAccZ / 4096.0 + 0.03;

  GyroX = (float)rawGyroX;
  GyroY = (float)rawGyroY;
  GyroZ = (float)rawGyroZ;
}
