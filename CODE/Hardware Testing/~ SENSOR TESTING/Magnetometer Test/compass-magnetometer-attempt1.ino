#include <Wire.h>
#include <math.h> // For atan2 and PI

#define HMC5883L_ADDR 0x1E
#define DECLINATION_ANGLE 0.6167  // In degrees

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize HMC5883L
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); 
  Wire.write(0x18); // 1 sample avg, 75 Hz, normal measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); 
  Wire.write(0xA0); // Gain = 5 (default)
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); 
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}

void loop() {
  int16_t x, y, z;

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Starting register for X, Z, Y
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);

  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();

    float heading = calculateHeadingYForward(x, y);
    heading += DECLINATION_ANGLE; // Add local declination angle

    if (heading < 0) heading += 360.0;
    if (heading >= 360.0) heading -= 360.0;

    Serial.println(heading);
  }

  delay(100);
}

float calculateHeadingYForward(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x) * (180.0 / PI); // Convert to degrees

  heading -= 0;  // Adjust if sensor is rotated/mounted differently

  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;

  return heading;
}
