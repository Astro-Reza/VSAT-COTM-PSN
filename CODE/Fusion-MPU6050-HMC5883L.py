import smbus
import time
import math
# KODE Baca Elevasi (pakai MPU6050), Azimuth pakai HMC5883
# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

# HMC5883L Address (use correct one, default is 0x1E)
HMC5883L_ADDR = 0x1E

# Kalman filter class (unchanged)
class Kalman:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P = [[0, 0], [0, 0]]

    def update(self, new_angle, new_rate, dt):
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        y = new_angle - self.angle
        S = self.P[0][0] + self.R_measure
        K0 = self.P[0][0] / S
        K1 = self.P[1][0] / S

        self.angle += K0 * y
        self.bias += K1 * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K0 * P00_temp
        self.P[0][1] -= K0 * P01_temp
        self.P[1][0] -= K1 * P00_temp
        self.P[1][1] -= K1 * P01_temp

        return self.angle

# port I2C
bus = smbus.SMBus(3) # ini cek dulu pake ls /dev/i2c* terus ke sudo i2cdetect -y [ini nomor brp? biasanya 3]

# MPU6050 setup
def mpu6050_setup():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)  # sensitivitas dari ±2g sampai ±8g, ganti address kalau mau makin sensitif (liat datasheet MPU6050)
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x08)   # ±500°/s

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def read_mpu6050():
    rawAccX = read_raw_data(0x3B)
    rawAccY = read_raw_data(0x3D)
    rawAccZ = read_raw_data(0x3F)
    rawGyroX = read_raw_data(0x43)
    rawGyroY = read_raw_data(0x45)
    rawGyroZ = read_raw_data(0x47)

    AccX = rawAccX / 4096.0 - 0.07
    AccY = rawAccY / 4096.0 + 0.01
    AccZ = rawAccZ / 4096.0 + 0.03

    GyroX = float(rawGyroX)
    GyroY = float(rawGyroY)
    GyroZ = float(rawGyroZ)

    return AccX, AccY, AccZ, GyroX, GyroY, GyroZ

# HMC5883L setup
def hmc5883l_setup():
    # Continuous measurement mode
    bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)  # Configuration Register A: 8-average, 15 Hz default
    bus.write_byte_data(HMC5883L_ADDR, 0x01, 0xA0)  # Configuration Register B: Gain = 5
    bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)  # Mode Register: Continuous measurement mode

def read_hmc5883l():
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    x = data[0] << 8 | data[1]
    z = data[2] << 8 | data[3]
    y = data[4] << 8 | data[5]

    # Handle signed values
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536

    return x, y, z

# \/\/\/ program utama dari sini \/\/\/
kalmanRoll = Kalman()
kalmanPitch = Kalman()

mpu6050_setup()
hmc5883l_setup()
time.sleep(0.5)

last_time = time.time()

try:
    while True:
        AccX, AccY, AccZ, GyroX, GyroY, GyroZ = read_mpu6050()
        magX, magY, magZ = read_hmc5883l()

        now = time.time()
        dt = now - last_time
        last_time = now

        gyroXrate = GyroX / 65.5
        gyroYrate = GyroY / 65.5

        accRoll = math.atan2(AccY, AccZ) * 180 / math.pi
        accPitch = math.atan2(-AccX, math.sqrt(AccY**2 + AccZ**2)) * 180 / math.pi

        AngleRoll = kalmanRoll.update(accRoll, gyroXrate, dt)
        AnglePitch = kalmanPitch.update(accPitch, gyroYrate, dt)

        print(f"{accRoll:.2f},{accPitch:.2f},{AngleRoll:.2f},{AnglePitch:.2f} | MagX: {magX} MagY: {magY} MagZ: {magZ}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nProgram stopped.")
