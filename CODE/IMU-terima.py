import socket
import matplotlib.pyplot as plt
from collections import deque

HOST = '10.32.211.113'  # IP of your Orange Pi
PORT = 12345

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((HOST, PORT))

roll_data = deque([0]*100, maxlen=100)
pitch_data = deque([0]*100, maxlen=100)

plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot(roll_data, label='Roll')
line2, = ax.plot(pitch_data, label='Pitch')
ax.set_ylim(-180, 180)
plt.legend()

try:
    while True:
        data = client.recv(1024).decode().strip()
        if data:
            try:
                roll, pitch = map(float, data.split(','))
                roll_data.append(roll)
                pitch_data.append(pitch)

                line1.set_ydata(roll_data)
                line2.set_ydata(pitch_data)
                line1.set_xdata(range(len(roll_data)))
                line2.set_xdata(range(len(pitch_data)))
                ax.relim()
                ax.autoscale_view()
                plt.pause(0.01)
            except ValueError:
                continue

except KeyboardInterrupt:
    print("Stopping client.")
    client.close()
