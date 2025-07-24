import time
import numpy as np
from smbus2 import SMBus
from filterpy.kalman import KalmanFilter
from math import atan2, degrees, radians, sin, cos, sqrt

# Sensor I2C addresses
MPU6050_ADDR = 0x68
HMC5883L_ADDR = 0x1E

bus = SMBus(1)

# Kalman filters
kf_heading = KalmanFilter(dim_x=2, dim_z=1)  # [yaw, yaw_rate]
kf_sideslip = KalmanFilter(dim_x=4, dim_z=2) # [v_long, v_lat, bias_ax, bias_ay]

def init_MPU6050():
    bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)  # Wake up
    time.sleep(0.1)

def read_MPU6050():
    data = bus.read_i2c_block_data(MPU6050_ADDR, 0x3B, 14)
    ax = twos_complement(data[0] << 8 | data[1]) / 16384.0
    ay = twos_complement(data[2] << 8 | data[3]) / 16384.0
    az = twos_complement(data[4] << 8 | data[5]) / 16384.0
    gz = twos_complement(data[8] << 8 | data[9]) / 131.0
    return ax, ay, az, gz

def init_HMC5883L():
    bus.write_byte_data(HMC5883L_ADDR, 0x00, 0x70)  # 8-average, 15 Hz
    bus.write_byte_data(HMC5883L_ADDR, 0x01, 0xA0)  # Gain
    bus.write_byte_data(HMC5883L_ADDR, 0x02, 0x00)  # Continuous

def read_HMC5883L():
    data = bus.read_i2c_block_data(HMC5883L_ADDR, 0x03, 6)
    mx = twos_complement(data[0] << 8 | data[1]) * 0.92
    mz = twos_complement(data[2] << 8 | data[3]) * 0.92
    my = twos_complement(data[4] << 8 | data[5]) * 0.92
    return mx, my, mz

def twos_complement(val):
    return val - 65536 if val > 32767 else val

def magnetic_heading(mx, my):
    heading = atan2(my, mx)
    if heading < 0:
        heading += 2 * np.pi
    return heading

def heading_kalman_predict(dt, gyro_z):
    # x = [yaw, bias]
    kf_heading.F = np.array([[1, -dt],
                             [0, 1]])
    kf_heading.H = np.array([[1, 0]])
    kf_heading.Q = np.eye(2) * 0.001
    kf_heading.R = np.array([[0.1]])
    kf_heading.predict()
    kf_heading.update(gyro_z)

def heading_kalman_correct(mag_heading, mag_norm, threshold=1.2):
    if abs(mag_norm - 1.0) < threshold:
        kf_heading.update(mag_heading)

def sideslip_kalman_predict(dt, ax, ay):
    kf_sideslip.F = np.array([
        [1, 0, -dt,  0],
        [0, 1,  0, -dt],
        [0, 0,  1,   0],
        [0, 0,  0,   1]
    ])
    kf_sideslip.Q = np.eye(4) * 0.05
    kf_sideslip.predict()

def sideslip_kalman_correct(v_long_gps, v_lat_gps):
    z = np.array([v_long_gps, v_lat_gps])
    kf_sideslip.H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    kf_sideslip.R = np.diag([0.2, 0.2])
    kf_sideslip.update(z)

def get_sideslip_angle():
    v_long, v_lat = kf_sideslip.x[0], kf_sideslip.x[1]
    return degrees(atan2(v_lat, v_long))

def setup_filters():
    # Initial values
    kf_heading.x = np.array([0.0, 0.0])
    kf_heading.P *= 1

    kf_sideslip.x = np.zeros(4)
    kf_sideslip.P *= 1

def main_loop():
    init_MPU6050()
    init_HMC5883L()
    setup_filters()

    last_time = time.time()

    while True:
        now = time.time()
        dt = now - last_time
        last_time = now

        ax, ay, az, gz = read_MPU6050()
        mx, my, mz = read_HMC5883L()
        mag_norm = sqrt(mx**2 + my**2 + mz**2)

        mag_heading = magnetic_heading(mx, my)
        heading_kalman_predict(dt, gz)
        heading_kalman_correct(mag_heading, mag_norm)

        # Replace with real GPS velocity in global frame
        gps_speed = 5.0  # m/s
        gps_course = radians(30.0)  # example
        est_heading = kf_heading.x[0]
        v_long = gps_speed * cos(gps_course - est_heading)
        v_lat  = gps_speed * sin(gps_course - est_heading)

        sideslip_kalman_predict(dt, ax, ay)
        sideslip_kalman_correct(v_long, v_lat)

        beta = get_sideslip_angle()

        print(f"Sideslip Angle β: {beta:.2f} deg  | Yaw: {degrees(est_heading):.2f} deg")

        time.sleep(0.05)

if __name__ == "__main__":
    main_loop()
