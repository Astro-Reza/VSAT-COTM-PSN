# MPC-Aided PID Stabilization Simulation

This simulation combines Kalman filtering, Model Predictive Control (MPC)-like prediction, PID control, and tolerance quantization to simulate and stabilize the pitch and roll of a system such as a drone or gimbal. It includes both 3D orientation visualization and 2D pitch tracking over time.

## Features 
- Kalman filter for sensor fusion (accelerometer + gyroscope).
- Predictive error estimation using future-state extrapolation (MPC-style).
- PID controller for simulated stabilization.
- Discrete-step rounding (tolerance) to simulate quantized sensor/control input.

### 3d Quaternion Visualization
*Relative*:  Estimated orientation.
*Level*: Reference level plane.
*Stabilized*: Actual system output after stabilization.

### 2d time-series pitch plot comparing:
- Kalman-estimated pitch (Measured).
- MPC-predicted pitch.
- PID-controlled system output.

## Key Components
### Kalman
Lightweight 1D Kalman filter for angle estimation (used for pitch and roll separately).

### sim_read_MPU6050()
Returns noisy synthetic pitch/roll values and gyro rates to simulate real IMU sensor input.

### calculate_error()
Predicts the future orientation based on recent change rates (using 4-point rate estimation when available). Output is bounded using np.clip.

### PID
Standard PID controller implementation.

### main()
- Applies Kalman filter to simulated sensor data.
- Rounds pitch/roll to simulate discrete sensor/actuator steps.
- Uses MPC-like error prediction as PID input.
- Applies PID correction to "actual" system angle.
- Stores history for visualization.

### animate()
Animates:
- *Top row (3D)*: Rotation of body frame using XYZ axis quivers
- *Bottom row (2D)*: Line plot of pitch: Measured vs Predicted vs Output