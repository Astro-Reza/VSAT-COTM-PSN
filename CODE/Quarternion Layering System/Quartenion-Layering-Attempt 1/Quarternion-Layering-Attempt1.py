import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
import time

# ----------------------
# Kalman filter (unchanged)
# ----------------------
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
        # Prediction
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
        self.P[0, 0] += dt * (dt * self.P[1, 1] - self.P[0, 1] - self.P[1, 0] + self.Q_angle)
        self.P[0, 1] -= dt * self.P[1, 1]
        self.P[1, 0] -= dt * self.P[1, 1]
        self.P[1, 1] += self.Q_bias * dt

        # Update
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

# ----------------------
# Simulation of MPU6050
# ----------------------
def sim_read_MPU6050():
    pitch = np.random.uniform(-30, 30) + np.random.randn() * 0.5
    roll = np.random.uniform(-30, 30) + np.random.randn() * 0.5
    gyro_pitch_rate = np.random.uniform(-100, 100)
    gyro_roll_rate = np.random.uniform(-100, 100)
    return pitch, roll, gyro_pitch_rate, gyro_roll_rate

# ----------------------
# MPC Error Prediction
# ----------------------
def calculate_error(filtered_angle, history, dt, pred_time=0.3):
    if len(history) < 2:
        rate = 0.0
    else:
        rate = (history[-1] - history[-2]) / dt
    predicted = filtered_angle + rate * pred_time
    error = -predicted
    return error

# ----------------------
# PID Controller
# ----------------------
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

# ----------------------
# Main Simulation Loop
# ----------------------
def main(sim_time=10.0, dt=0.01):
    kalman_pitch = Kalman()
    kalman_roll = Kalman()
    pid_pitch = PID(kp=2.0, ki=0.1, kd=0.5)
    pid_roll = PID(kp=2.0, ki=0.1, kd=0.5)
    history_pitch = []
    history_roll = []

    q_rel = []
    q_level = []
    q_pid = []

    t = 0.0
    while t < sim_time:
        acc_pitch, acc_roll, gyr_pr, gyr_rr = sim_read_MPU6050()
        angle_pitch = kalman_pitch.update(acc_pitch, gyr_pr, dt)
        angle_roll  = kalman_roll.update(acc_roll, gyr_rr, dt)
        history_pitch.append(angle_pitch)
        history_roll.append(angle_roll)
        err_pitch = calculate_error(angle_pitch, history_pitch, dt)
        err_roll  = calculate_error(angle_roll, history_roll, dt)
        corr_pitch = pid_pitch.update(err_pitch, dt)
        corr_roll  = pid_roll.update(err_roll, dt)
        q_rel.append(R.from_euler('yx', [angle_pitch, angle_roll], degrees=True))
        q_level.append(R.from_euler('yx', [0, 0], degrees=True))
        q_pid.append(R.from_euler('yx', [-corr_pitch, -corr_roll], degrees=True))
        t += dt

    animate(quaternions=[q_rel, q_level, q_pid], dt=dt)

# ----------------------
# Visualization Animation
# ----------------------
def animate(quaternions, dt):
    labels = ['Relative', 'Level', 'PID Stabilized']
    fig = plt.figure(figsize=(12, 4))
    axes = [fig.add_subplot(1, 3, i+1, projection='3d') for i in range(3)]

    sphere = []
    quivers = []

    for ax, label in zip(axes, labels):
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

    def update(frame):
        for i in range(3):
            rot = quaternions[i][frame].as_matrix()
            for vec, qv, col in zip(rot.T, quivers[i], ['r', 'g', 'b']):
                qv.remove()
                quivers[i][['r', 'g', 'b'].index(col)] = axes[i].quiver(0, 0, 0, *vec, color=col, length=0.8)
        return sum(quivers, [])

    ani = FuncAnimation(fig, update, frames=len(quaternions[0]), interval=dt*1000, blit=False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()