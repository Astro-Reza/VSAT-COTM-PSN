#include <Wire.h>
#include <math.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float SmoothedRoll = 0;
float SmoothedPitch = 0;
float LoopTimer;
const float alpha = 0.3;  // Smoothing factor, between 0 (more smooth) and 1 (no smoothing)

void gyro_signals(void) {
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

  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Convert to g-forces (assuming ±8g → 4096 LSB/g)
  AccX = (float)AccXLSB / 4096.0 - 0.07;
  AccY = (float)AccYLSB / 4096.0 + 0.01;
  AccZ = (float)AccZLSB / 4096.0 + 0.03;

  // Calculate raw tilt angles
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);
  AnglePitch = atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);

  // Apply exponential interpolation (smoothing)
  SmoothedRoll = alpha * AngleRoll + (1 - alpha) * SmoothedRoll;
  SmoothedPitch = alpha * AnglePitch + (1 - alpha) * SmoothedPitch;
}

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  Wire.begin();
  Wire.setClock(400000);  // Fast I2C
  delay(250);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  gyro_signals();
  Serial.print("Raw Roll [°]: ");
  Serial.print(AngleRoll);
  Serial.print(", Smoothed Roll [°]: ");
  Serial.print(SmoothedRoll);

  Serial.print(" | Raw Pitch [°]: ");
  Serial.print(AnglePitch);
  Serial.print(", Smoothed Pitch [°]: ");
  Serial.println(SmoothedPitch);

  delay(50);
}
