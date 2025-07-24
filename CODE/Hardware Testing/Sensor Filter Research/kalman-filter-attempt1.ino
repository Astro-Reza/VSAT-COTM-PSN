#include <Wire.h>
#include <math.h>

// Gyroscope and Accelerometer readings
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Calibration variables
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
int RateCalibrationSamples = 2000;

// Kalman filter variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4;
float Kalman1DOutput[] = {0, 0};

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;  // Predict angle
  KalmanUncertainty = KalmanUncertainty + 0.004 * -0.004 * 16; // Predict uncertainty
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); // Update step
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void gyro_signals(void) {
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw data to physical units
  RateRoll = (float)GyroX / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
  RateYaw = (float)GyroZ / 65.5 - RateCalibrationYaw;

  AccX = (float)AccXLSB / 4096.0 - 0.07;
  AccY = (float)AccYLSB / 4096.0 + 0.01;
  AccZ = (float)AccZLSB / 4096.0 + 0.03;

  // Angle calculations in degrees
  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
  AnglePitch = atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;
}

void calibrateGyro() {
  long sumRoll = 0, sumPitch = 0, sumYaw = 0;
  for (int i = 0; i < RateCalibrationSamples; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 6);
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    sumRoll += GyroX;
    sumPitch += GyroY;
    sumYaw += GyroZ;
    delay(1);
  }
  RateCalibrationRoll = (float)sumRoll / RateCalibrationSamples / 65.5;
  RateCalibrationPitch = (float)sumPitch / RateCalibrationSamples / 65.5;
  RateCalibrationYaw = (float)sumYaw / RateCalibrationSamples / 65.5;
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

  // Configure accelerometer (±8g)
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Configure gyroscope (±500°/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Calibrate gyro
  Serial.println("Calibrating gyroscope...");
  calibrateGyro();
  Serial.println("Calibration done.");
}

void loop() {
  gyro_signals();

  // Kalman filtering
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll = Kalman1DOutput[0];
  KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch = Kalman1DOutput[0];
  KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

  // Print
  Serial.print("AccX = "); Serial.print(AccX);
  Serial.print(", AccY = "); Serial.print(AccY);
  Serial.print(", AccZ = "); Serial.print(AccZ);
  Serial.print(", Kalman Roll = "); Serial.print(KalmanAngleRoll);
  Serial.print(", Kalman Pitch = "); Serial.println(KalmanAnglePitch);

  delay(50);
}
