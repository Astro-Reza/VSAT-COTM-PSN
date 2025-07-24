#include <Wire.h>
#include <math.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float LoopTimer;

void gyro_signals(void) {
  // Configure low-pass filter
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Configure accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10); // ±8g range
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
  AccX = (float)AccXLSB / 4096.0-0.08;
  AccY = (float)AccYLSB / 4096.0;
  AccZ = (float)AccZLSB / 4096.0+ 0.09;

  // Calculate tilt angles
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
}

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  Wire.begin();              // Arduino Nano: A4 (SDA), A5 (SCL)
  Wire.setClock(400000);     // 400kHz I2C speed
  delay(250);

  // Wake up MPU6050 (disable sleep mode)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  gyro_signals();
  Serial.print(", Angle Roll [°] = ");
  Serial.print(AngleRoll);
  Serial.print(", Angle Pitch [°] = ");
  Serial.print(AnglePitch);
  Serial.print(" ------- ");
  Serial.print(AccX);
  Serial.print("|");
  Serial.print(AccY);
  Serial.print("|");
  Serial.println(AccZ); 
  delay(50);
}
