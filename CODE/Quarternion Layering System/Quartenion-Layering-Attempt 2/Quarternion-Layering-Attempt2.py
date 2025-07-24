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
        self.rate = new_rate - self.bias
        self.angle += dt * self.rate
        self.P[0, 0] += dt * (dt * self.P[1, 1] - self.P[0, 1] - self.P[1, 0] + self.Q_angle)
        self.P[0, 1] -= dt * self.P[1, 1]
        self.P[1, 0] -= dt * self.P[1, 1]
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

# ----------------------
# Simulation of MPU6050
# ----------------------
def sim_read_MPU6050():
    pitch = np.random.uniform(-30, 30) + np.random.randn() * 0.5
    roll = np.random.uniform(-30, 30) + np.random.randn() * 0.5
    gyro_pitch_rate = np.random.uniform(-100, 100)
    gyro_roll_rate = np.random.uniform(-100, 100)
    return pitch, roll, gyro_pitch_rate, gyro_roll_rate

# MPC Error Prediction ----------------------
def calculate_error(filtered_angle, history, dt, pred_time=0.1, max_pred=10.0, scale=1.0):

    
    if len(history) >= 4: # ambil 4 point
        rate = (history[-1] - history[-4]) / (3 * dt)
    elif len(history) >= 2:
        rate = (history[-1] - history[-2]) / dt
    else:
        rate = 0.0

    predicted = filtered_angle + rate * pred_time
    predicted = np.clip(predicted, -max_pred, max_pred)

    error = -predicted * scale
    return error, predicted

# PID Controller ----------------------
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

# Main Simulation Loop ----------------------
def main(sim_time=10.0, dt=0.05):
    kalman_pitch = Kalman()
    kalman_roll = Kalman()
    pid_pitch = PID(kp=0.95, ki=0.1, kd=0.5)
    pid_roll = PID(kp=0.90, ki=0.1, kd=0.5)
    history_pitch = []
    history_roll = []

    q_rel = []
    q_level = []
    q_actual = []

    pitch_data = []
    pitch_pred = []
    pitch_corr = []

    t = 0.0
    angle_pitch_actual = 0.0
    angle_roll_actual = 0.0

    while t < sim_time:
        acc_pitch, acc_roll, gyr_pr, gyr_rr = sim_read_MPU6050()
        angle_pitch = kalman_pitch.update(acc_pitch, gyr_pr, dt)
        angle_roll  = kalman_roll.update(acc_roll, gyr_rr, dt)

        # Apply tolerance
        def apply_tolerance(x, step=1.0):
            return round(x / step) * step

        angle_pitch = apply_tolerance(angle_pitch)
        angle_roll = apply_tolerance(angle_roll)
        history_pitch.append(angle_pitch)
        history_roll.append(angle_roll)

        err_pitch, predicted_pitch = calculate_error(angle_pitch, history_pitch, dt)
        err_roll, predicted_roll = calculate_error(angle_roll, history_roll, dt)

        corr_pitch = pid_pitch.update(err_pitch, dt)
        corr_roll  = pid_roll.update(err_roll, dt)

        # simulate actual response by moving toward correction
        angle_pitch_actual += corr_pitch * dt
        angle_roll_actual  += corr_roll * dt

        q_rel.append(R.from_euler('yx', [angle_pitch, angle_roll], degrees=True))
        q_level.append(R.from_euler('yx', [0, 0], degrees=True))
        q_actual.append(R.from_euler('yx', [angle_pitch_actual, angle_roll_actual], degrees=True))

        pitch_data.append(angle_pitch)
        pitch_pred.append(predicted_pitch)
        pitch_corr.append(angle_pitch_actual)

        t += dt

    animate([q_rel, q_level, q_actual], dt, pitch_data, pitch_pred, pitch_corr)

# ----------------------
# Visualization Animation
# ----------------------
def animate(quaternions, dt, pitch_data, pitch_pred, pitch_corr):
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
    ax2d.set_xlim(0, len(pitch_data))
    ax2d.set_ylim(-40, 40)
    ax2d.set_ylabel("Pitch (degrees)")
    ax2d.legend()

    def update(frame):
        for i in range(3):
            rot = quaternions[i][frame].as_matrix()
            for vec, qv, col in zip(rot.T, quivers[i], ['r', 'g', 'b']):
                qv.remove()
                quivers[i][['r', 'g', 'b'].index(col)] = axes3d[i].quiver(0, 0, 0, *vec, color=col, length=0.8)
        x = list(range(frame))
        line_measured.set_data(x, pitch_data[:frame])
        line_pred.set_data(x, pitch_pred[:frame])
        line_actual.set_data(x, pitch_corr[:frame])
        return sum(quivers, []) + [line_measured, line_pred, line_actual]

    ani = FuncAnimation(fig, update, frames=len(quaternions[0]), interval=dt*1000*4, blit=False)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()