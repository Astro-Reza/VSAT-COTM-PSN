import socket
import smbus
import time
import math

# === Kalman Filter Class ===
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

# === MPU6050 I2C Setup ===
MPU6050_ADDR = 0x30
PWR_MGMT_1 = 0x6B
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B
ACCEL_XOUT_H = 0x3B

bus = smbus.SMBus(2)

def mpu6050_setup():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x11)  # ±8g
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

# === Socket Setup ===
HOST = '0.0.0.0'
PORT = 12345

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind((HOST, PORT))
server.listen(1)
print("Waiting for client to connect...")
conn, addr = server.accept()
print(f"Connected by {addr}")

# === Kalman Filters & Sensor Init ===
kalmanRoll = Kalman()
kalmanPitch = Kalman()
mpu6050_setup()
time.sleep(0.5)
last_time = time.time()

# === Main Loop ===
try:
    while True:
        AccX, AccY, AccZ, GyroX, GyroY, GyroZ = read_mpu6050()
        now = time.time()
        dt = now - last_time
        last_time = now

        gyroXrate = GyroX / 65.5
        gyroYrate = GyroY / 65.5

        accRoll = math.atan2(AccY, AccZ) * 180 / math.pi
        accPitch = math.atan2(-AccX, math.sqrt(AccY**2 + AccZ**2)) * 180 / math.pi

        AngleRoll = kalmanRoll.update(accRoll, gyroXrate, dt)
        AnglePitch = kalmanPitch.update(accPitch, gyroYrate, dt)

        data_str = f"{AngleRoll:.2f},{AnglePitch:.2f}\n"
        conn.sendall(data_str.encode())
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopping server.")
    conn.close()
    server.close()
