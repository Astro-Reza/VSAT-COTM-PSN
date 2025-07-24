# Orientation Estimation and Stabilization Simulation

This project simulates orientation estimation using a Kalman Filter, stabilization using a PID controller, and visualizes the resulting orientation using 3D animation. It mimics how a self-balancing system such as a drone or gimbal might respond to simulated motion inputs from an MPU6050-like sensor.

## Features
- Kalman filter implementation for sensor fusion (accelerometer + gyroscope)
- Simulated MPU6050 sensor readings
- MPC-inspired future-state error prediction
- PID controller for orientation correction
- Quaternion-based orientation tracking
- 3D visualization using matplotlib.animation

## Core Components
### 1. Kalman
- A simple 1d Kalman Filter used independently for pitch and roll estimation.

### 2. PID
A basic PID controller used to correct predicted orientation error.

### 3. sim_read_MPU6050()
- Simulates readings from an MPU6050 IMU:
- Returns random pitch/roll values with noise and simulated gyro rates.

### 4. calculate_error()
- Predicts the future angle using current Kalman-filtered angle and recent change rate. Returns an error signal for the PID controller.

### 5. main()
#### Main simulation loop:
- Runs a virtual time loop
- Applies Kalman filter on simulated IMU data
- Feeds predicted error to PID controller
#### Generates three orientation quaternions:
- q_rel: estimated orientation.
- q_level: reference level (flat).
- q_pid: orientation after PID correction.

### 6. animate()
Visualizes the three orientations using rotating coordinate frames (quaternions) in 3D subplots.

## Output Visualization
*Relative*: Orientation estimated by Kalman Filter
*Level*: Flat reference plane (ideal)
*PID Stabilized*: Resulting orientation after PID correction.