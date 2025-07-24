import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import time
import serial
import threading

# Kalman filter (unchanged) ------------------
class Kalman:
    def __init__(self):
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

        self.angle = 0.0
        self.bias = 0.0
        self.rate = 0.0
        self.P = np.zeros((2, 2))

    def update(self, new_angle, new_rate, dt):
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
        self.P[0, 0] += dt * (dt * self.P[1, 1] - self.P[0, 1] - self.P[1, 0] + self.Q_angle)
        self.P[0, 1] -= dt * self.P[1, 1]
        self.P[1, 0] -= self.P[1, 1]
        self.P[1, 1] += self.Q_bias * dt

        y = new_angle - self.angle
        S = self.P[0, 0] + self.R_measure
        K = np.array([self.P[0, 0], self.P[1, 0]]) / S
        self.angle += K[0] * y
        self.bias += K[1] * y

        P00, P01 = self.P[0, 0], self.P[0, 1]
        self.P[0, 0] -= K[0] * P00
        self.P[0, 1] -= K[0] * P01
        self.P[1, 0] -= K[1] * P00
        self.P[1, 1] -= K[1] * P01

        return self.angle

# Read from serial (COM9) - only x and y values (roll and pitch)------------------
def sim_read_MPU6050(ser):
    try:
        line = ser.readline().decode().strip()
        values = [v.strip() for v in line.split(',') if v.strip() != '']

        if len(values) < 2:
            raise ValueError("Expected at least 2 comma-separated values, got:", line)

        roll = float(values[0])   # x
        pitch = float(values[1])  # y
        return pitch, roll

    except Exception as e:
        print("Sensor read error:", e)
        return 0.0, 0.0

# MPC-like prediction
def calculate_mpc_prediction(filtered_angle, history, dt, pred_time=0.2):
    if len(history) >= 2:
        rate = (history[-1] - history[-2]) / dt
    else:
        rate = 0.0
    predicted = filtered_angle + rate * pred_time
    return predicted

# Simple PID controller
def pid_control(setpoint, current, prev_error, integral, kp=2.8, ki=0.0, kd=0.0):
    error = setpoint - current
    integral += error
    derivative = error - prev_error
    output = kp * error + ki * integral + kd * derivative
    return output, error, integral

# Live Visualization (roll and pitch from x and y)
def live_main(dt=0.05):
    kalman_pitch = Kalman()
    kalman_roll = Kalman()

    pitch_data = []
    pitch_pred = []
    pitch_corr = []

    angle_pitch_actual = 0.0
    angle_roll_actual = 0.0

    pid_error = 0.0
    pid_integral = 0.0

    q_rel, q_level, q_actual = [], [], []

    ser = serial.Serial('COM9', 115200, timeout=0.05)
    time.sleep(2)

    labels = ['Relative', 'Level', 'Stabilized']
    fig = plt.figure(figsize=(12, 8))
    axes3d = [fig.add_subplot(2, 3, i+1, projection='3d') for i in range(3)]
    ax2d = fig.add_subplot(2, 1, 2)

    quivers = []
    for ax, label in zip(axes3d, labels):
        u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
        x = np.cos(u)*np.sin(v)
        y = np.sin(u)*np.sin(v)
        z = np.cos(v)
        ax.plot_wireframe(x, y, z, alpha=0.2)
        qv = [ax.quiver(0, 0, 0, 1, 0, 0), ax.quiver(0, 0, 0, 0, 1, 0), ax.quiver(0, 0, 0, 0, 0, 1)]
        quivers.append(qv)
        ax.set_title(label)
        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
        ax.set_axis_off()

    line_measured, = ax2d.plot([], [], label='Measured', color='blue')
    line_pred, = ax2d.plot([], [], label='MPC Predicted', color='orange')
    line_actual, = ax2d.plot([], [], label='System Output', color='green')
    ax2d.set_ylim(-40, 40)
    ax2d.set_ylabel("Pitch (degrees)")
    ax2d.legend()

    def update(frame):
        nonlocal angle_pitch_actual, angle_roll_actual, pid_error, pid_integral
        acc_pitch, acc_roll = sim_read_MPU6050(ser)

        angle_pitch = kalman_pitch.update(acc_pitch, 0.0, dt)
        angle_roll = kalman_roll.update(acc_roll, 0.0, dt)

        angle_pitch = round(angle_pitch, 1)
        angle_roll = round(angle_roll, 1)

        pitch_data.append(angle_pitch)
        predicted_pitch = calculate_mpc_prediction(angle_pitch, pitch_data, dt)
        pitch_pred.append(predicted_pitch)

        # PID controller updates actual pitch based on predicted target
        control_output, pid_error, pid_integral = pid_control(predicted_pitch, angle_pitch_actual, pid_error, pid_integral)
        angle_pitch_actual += control_output * dt  # Apply control to simulated actuator

        pitch_corr.append(angle_pitch_actual)

        q_rel.append(R.from_euler('yx', [angle_pitch, angle_roll], degrees=True))
        q_level.append(R.from_euler('yx', [0, 0], degrees=True))
        q_actual.append(R.from_euler('yx', [angle_pitch_actual, angle_roll_actual], degrees=True))

        for i in range(3):
            rot = [q_rel, q_level, q_actual][i][-1].as_matrix()
            for vec, qv, col in zip(rot.T, quivers[i], ['r', 'g', 'b']):
                qv.remove()
                quivers[i][['r', 'g', 'b'].index(col)] = axes3d[i].quiver(0, 0, 0, *vec, color=col, length=0.8)

        x = list(range(len(pitch_data)))
        line_measured.set_data(x, pitch_data)
        line_pred.set_data(x, pitch_pred)
        line_actual.set_data(x, pitch_corr)
        ax2d.set_xlim(0, len(pitch_data))
        return sum(quivers, []) + [line_measured, line_pred, line_actual]

    ani = FuncAnimation(fig, update, interval=dt*1000, blit=False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    live_main()
