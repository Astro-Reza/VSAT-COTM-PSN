## This code used to test DM556 connection using DIR and PUL pins.

DIR: Direction of the motor.
PUL: Same as STEP in TMC2208 or any other stepper motor. Determine how many steps does the motor needs to take.

Each negative connection should be connected into one in series. i.e. DIR-, PUL-, to the GND.

# DM556 has different microsteps resolutions:
400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 1000, 2000, 5000, 10000, 25000, 50000.

- Use higher microsteps when you need smooth, precise, and quiet motion.
- Use lower microsteps when you need more torque or higher speed, and resolution is less critical.

### Higher Microsteps Means:
- Smoother motion: Reduces vibration and noise, ideal for precision systems like CNCs, cameras, or antennas.
- More precise positioning: Each step is smaller, increasing resolution.
- Reduced mechanical resonance: Helps prevent issues at certain speeds due to smoother transitions.
- Lower torque per microstep: Each step delivers less torque. 
- May exceed controller resolution: Your microcontroller must send more steps per movement, which might be a bottleneck.
- Higher computational load if the pulse generator can't keep up.

### Lower Microsteps Means:
- Stronger torque per step: Each step delivers more power.
- Easier to reach higher speeds: Fewer steps per revolution means faster rotation possible with limited pulse rates.
- Less demanding on the controller: Fewer pulses to generate.
- Rougher movement: More vibration, especially at low speeds.
- Lower resolution: Not suitable where precise positioning is critical.