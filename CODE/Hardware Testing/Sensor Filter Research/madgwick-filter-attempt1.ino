#include <Wire.h>
#include <MadgwickAHRS.h>
#include <math.h>

Madgwick filter;

float ax, ay, az;
float gx, gy, gz;
unsigned long lastTime = 0;
float deltat;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  delay(100);

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

  // Configure gyroscope (±500 °/s)
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  lastTime = micros();
}

void loop() {
  readMPU6050();

  // Calculate delta time
  unsigned long now = micros();
  deltat = (now - lastTime) / 1000000.0;
  lastTime = now;

  // Update Madgwick filter
  filter.updateIMU(gx, gy, gz, ax, ay, az);

  // Get orientation (in degrees)
  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw();

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" | Pitch: ");
  Serial.print(pitch);
  Serial.print(" | Yaw: ");
  Serial.println(yaw);

  delay(50);
}

void readMPU6050() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 14, true);

  int16_t rawAccX = Wire.read() << 8 | Wire.read();
  int16_t rawAccY = Wire.read() << 8 | Wire.read();
  int16_t rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature
  int16_t rawGyroX = Wire.read() << 8 | Wire.read();
  int16_t rawGyroY = Wire.read() << 8 | Wire.read();
  int16_t rawGyroZ = Wire.read() << 8 | Wire.read();

  // Convert to physical units
  ax = (float)rawAccX / 4096.0; // ±8g = 4096 LSB/g
  ay = (float)rawAccY / 4096.0;
  az = (float)rawAccZ / 4096.0;

  gx = (float)rawGyroX / 65.5 * DEG_TO_RAD; // ±500°/s = 65.5 LSB/°/s
  gy = (float)rawGyroY / 65.5 * DEG_TO_RAD;
  gz = (float)rawGyroZ / 65.5 * DEG_TO_RAD;
}
