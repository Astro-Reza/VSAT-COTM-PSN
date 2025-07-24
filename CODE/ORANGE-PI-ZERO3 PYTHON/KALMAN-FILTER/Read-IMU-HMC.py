import time
import numpy as np
import smbus  # Using smbus instead of smbus2
from filterpy.kalman import KalmanFilter
from math import atan2, degrees, radians, sqrt

# I2C setup
MPU6050_ADDR = 0x68
HMC5883L_ADDR = 0x1E
bus = smbus.SMBus(1)  # I2C-1 bus on Orange Pi

kf_heading = KalmanFilter(dim_x=2, dim_z=1)  # [yaw, bias]

# ---- Setup and Utilities ----

def twos_complement(val):
    return val - 65536 if val > 32767 else val

def init_MPU6050():
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
    time.sleep(0.1)

def read_MPU6050():
    data = bus.read_i2c_block_data(MPU6050_ADDR, 0x3B, 14)
    gz = twos_complement(data[8] << 8 | data[9]) / 131.0  # gyro z in deg/s
    return gz

def init_HMC5883L():
    bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)  # Config A
    bus.write_byte_data(HMC5883L_ADDR, 0x01, 0xA0)  # Gain
    bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)  # Continuous mode

def read_HMC5883L():
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    mx = twos_complement(data[0] << 8 | data[1]) * 0.92
    mz = twos_complement(data[2] << 8 | data[3]) * 0.92
    my = twos_complement(data[4] << 8 | data[5]) * 0.92
    return mx, my, mz

def magnetic_heading(mx, my):
    heading = atan2(my, mx)
    if heading < 0:
        heading += 2 * np.pi
    return heading

def magnetic_norm(mx, my, mz):
    return sqrt(mx**2 + my**2 + mz**2) / 1000.0  # normalize to ~1

def setup_kf_heading():
    kf_heading.x = np.array([0.0, 0.0])  # [yaw, bias]
    kf_heading.P *= 1
    kf_heading.F = np.eye(2)
    kf_heading.H = np.array([[1, 0]])
    kf_heading.R = np.array([[0.1]])
    kf_heading.Q = np.eye(2) * 0.001

def heading_kalman_predict(dt, gyro_z):
    kf_heading.F = np.array([
        [1, -dt],
        [0,  1]
    ])
    kf_heading.predict(u=gyro_z)

def heading_kalman_correct(mag_heading, mag_norm, threshold=0.2):
    if abs(mag_norm - 1.0) < threshold:
        kf_heading.update(mag_heading)

# ---- Main Loop ----

def main_loop():
    init_MPU6050()
    init_HMC5883L()
    setup_kf_heading()
    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        gyro_z = read_MPU6050()
        mx, my, mz = read_HMC5883L()

        heading_kalman_predict(dt, radians(gyro_z))  # deg/s -> rad/s
        mag_heading = magnetic_heading(mx, my)
        norm = magnetic_norm(mx, my, mz)
        heading_kalman_correct(mag_heading, norm)

        estimated_yaw = kf_heading.x[0]
        print(f"Yaw Estimate: {degrees(estimated_yaw):.2f}° | Mag Norm: {norm:.3f}")

        time.sleep(0.05)

if __name__ == "__main__":
    main_loop()
