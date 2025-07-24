import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import time

# Setup figure and 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
plt.ion()  # Enable interactive mode

# Frame origin
origin = np.array([0, 0, 0])

# Oscillation parameters
frequency = 5  # Hz
amplitude_deg = 20  # Degrees
amplitude_rad = np.deg2rad(amplitude_deg)

start_time = time.time()

def draw_frame(R_matrix):
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Quaternion Orientation (Z fixed, XY vibrating)')

    # Draw coordinate frame arrows: Z = blue, Y = green, X = red
    for i, color in enumerate(['r', 'g', 'b']):
        ax.quiver(*origin, *R_matrix[:, i], color=color, linewidth=3)
    plt.draw()
    plt.pause(0.01)

# Animation loop
try:
    while plt.fignum_exists(fig.number):
        t = time.time() - start_time

        # Oscillating angles around X and Y (Z fixed)
        angle_x = amplitude_rad * np.sin(2 * np.pi * frequency * t)
        angle_y = amplitude_rad * np.cos(2 * np.pi * frequency * t)
        angle_z = 0  # No Z rotation

        # Create rotation from Euler angles
        rotation = R.from_euler('xyz', [angle_x, angle_y, angle_z])
        R_matrix = rotation.as_matrix()

        draw_frame(R_matrix)

except KeyboardInterrupt:
    print("Stopped by user")
