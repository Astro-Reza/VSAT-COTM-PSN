#include <Wire.h>
#include <math.h>

// Raw sensor readings
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Kalman filter variables
float kalAngleRoll = 0, kalAnglePitch = 0;
float kalmanRollP = 1, kalmanPitchP = 1;
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float biasRoll = 0, biasPitch = 0;

// Timing
unsigned long lastTime = 0;
float dt;

// Function to read IMU and compute raw angles
void gyro_signals() {
  // Configure low-pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer ±8g
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Read accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  AccX = (float)AccXLSB / 4096.0 - 0.07;
  AccY = (float)AccYLSB / 4096.0 + 0.01;
  AccZ = (float)AccZLSB / 4096.0 + 0.03;

  // Read gyroscope
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  // Accelerometer angle estimation
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;
}

float kalmanUpdate(float newAngle, float newRate, float& bias, float& angle, float& P) {
  // Prediction
  angle += (newRate - bias) * dt;
  P += Q_angle + Q_bias;

  // Measurement update
  float S = P + R_measure;
  float K = P / S;
  float y = newAngle - angle;
  angle += K * y;
  bias += K * y;
  P *= (1 - K);

  return angle;
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

  lastTime = micros();
}

void loop() {
  unsigned long now = micros();
  dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  gyro_signals();

  // Apply Kalman filter
  kalAngleRoll = kalmanUpdate(AngleRoll, RateRoll, biasRoll, kalAngleRoll, kalmanRollP);
  kalAnglePitch = kalmanUpdate(AnglePitch, RatePitch, biasPitch, kalAnglePitch, kalmanPitchP);

  Serial.print("Roll: ");
  Serial.print(kalAngleRoll);
  Serial.print("°, Pitch: ");
  Serial.println(kalAnglePitch);
  
  delay(50);
}
