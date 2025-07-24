#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define MPU6050_ADDR 0x68
#define HMC5883L_ADDR 0x1E

TinyGPSPlus gps;
SoftwareSerial gpsSerial(4, 3);  // RX, TX

float gps_speed = 0;
float gps_course = 0;
float heading_deg = 0;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin();

  initMPU6050();
  initHMC5883L();
}

void loop() {
  float ax, ay, az, gx, gy, gz;
  readMPU6050(ax, ay, az, gx, gy, gz);

  float mx, my, mz;
  readHMC5883L(mx, my, mz);

  // Calculate heading from magnetometer
  heading_deg = atan2(my, mx) * 180.0 / PI;
  if (heading_deg < 0) heading_deg += 360.0;

  readGPS();

  if (gps_speed > 0.5) {
    float course_rad = gps_course * PI / 180.0;
    float heading_rad = heading_deg * PI / 180.0;

    float v_long = gps_speed * cos(course_rad - heading_rad);
    float v_lat  = gps_speed * sin(course_rad - heading_rad);

    float beta = atan2(v_lat, v_long) * 180.0 / PI;

    Serial.print("Sideslip (beta): ");
    Serial.print(beta, 2);
    Serial.print(" deg  ");
  }

  Serial.println(heading_deg);

  delay(100);
}

void initMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // Power management
  Wire.write(0);     // Wake up
  Wire.endTransmission();
}

void readMPU6050(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14);

  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    raw[i] = (Wire.read() << 8) | Wire.read();
  }

  ax = raw[0] / 16384.0;
  ay = raw[1] / 16384.0;
  az = raw[2] / 16384.0;
  gx = raw[4] / 131.0;
  gy = raw[5] / 131.0;
  gz = raw[6] / 131.0;
}

void initHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); Wire.write(0x70); // 8 avg, 15 Hz, normal
  Wire.write(0x01); Wire.write(0xA0); // Gain
  Wire.write(0x02); Wire.write(0x00); // Continuous measurement
  Wire.endTransmission();
}

void readHMC5883L(float &mx, float &my, float &mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Data register
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 6);

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();

  mx = rawX * 0.92;  // mG/LSB
  my = rawY * 0.92;
  mz = rawZ * 0.92;
}

void readGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.speed.isUpdated()) {
    gps_speed = gps.speed.mps();
    gps_course = gps.course.deg();
  }
}
