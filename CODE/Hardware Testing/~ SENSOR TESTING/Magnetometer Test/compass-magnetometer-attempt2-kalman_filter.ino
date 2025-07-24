#include <Wire.h>

#define HMC5883L_ADDR 0x1E
#define DECLINATION_ANGLE 0.6167  // degrees

// === Kalman Filter Struct ===
typedef struct {
  float angle;
  float bias;
  float rate;
  float P[2][2];
  float Q_angle;
  float Q_bias;
  float R_measure;
} Kalman1D;

Kalman1D kalmanHeading;

// === Kalman Update Function ===
float kalmanHeadingUpdate(Kalman1D *kf, float newAngle, float newRate, float dt) {
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

  // Initialize Kalman filter
  kalmanHeading = {0, 0, 0, {{0, 0}, {0, 0}}, 0.0003, 0.003, 0.1};

  // Init HMC5883L
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); 
  Wire.write(0x18); // 1-average, 75 Hz, normal measurement
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); 
  Wire.write(0xA0); // Gain
  Wire.endTransmission();

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); 
  Wire.write(0x00); // Continuous measurement
  Wire.endTransmission();
}

void loop() {
  static unsigned long lastTime = micros();
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  int16_t x, y, z;

  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);

  if (Wire.available() == 6) {
    x = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
  }

  // Compute raw heading
  float rawHeading = calculateHeadingYForward(x, y);
  rawHeading += DECLINATION_ANGLE;
  if (rawHeading < 0) rawHeading += 360.0;
  if (rawHeading >= 360.0) rawHeading -= 360.0;

  // Compute angular rate
  static float lastRawHeading = rawHeading;
  float rate = (rawHeading - lastRawHeading) / dt;
  if (rate > 180) rate -= 360;
  if (rate < -180) rate += 360;
  lastRawHeading = rawHeading;

  // Apply Kalman filter
  float filteredHeading = kalmanHeadingUpdate(&kalmanHeading, rawHeading, rate, dt);

  Serial.print(rawHeading);
  Serial.print(",");
  Serial.println(filteredHeading);

  delay(20);
}

float calculateHeadingYForward(int16_t x, int16_t y) {
  float heading = atan2((float)y, (float)x) * (180.0 / PI);
  heading -= 160.0;  // custom mount/shift 
  if (heading < 0) heading += 360.0;
  return heading;
}
