import numpy as np
from scipy.integrate import cumtrapz
import matplotlib.pyplot as plt


# time = np.linspace(0, 10, 100)  # 100 points between 0 and 10 seconds
# acceleration = np.sin(time)  # Example: acceleration as a sine wave

# time = [0, 2]
# acceleration = [0, 5]

# # Generate more time points between 0 and 2
# time = np.linspace(0, 15, 100)
# # Corresponding acceleration (constant at 5 m/s^2)
# acceleration = np.where(time > 0, 5, 0)

# Generate time points between 0 and 12 seconds
# time = np.linspace(0, 12, 5000)  # More points for accuracy

# Define acceleration:
# First 2 seconds at 5 m/s^2, then 0 m/s^2 after that
# acceleration = np.where((time > 0) & (time <= 2), 5, 0)

time = [0,
        0.695435,
        0.847518,
        1.009681,
        1.159238,
        1.326339,
        1.473984,
        1.620297,
        1.759594,
        1.894865,
        2.03023,
        2.16013,
        2.29666,
        2.434143,
        2.568568,
        2.717522,
        2.849096,
        2.987806,
        3.124993,
        3.275661,
        3.427453,
        3.563472,
        3.702971,
        3.86407,
        4.010829,
        4.149331,
        4.289768,
        4.441539,
        4.577635,
        4.723208,
        4.858868
        ]

acceleration = [0.828,
                0.854,
                0.828,
                0.842,
                0.814,
                1.065,
                0.652,
                0.504,
                1.32,
                0.769,
                0.719,
                1.368,
                1.153,
                0.222,
                1.652,
                1.446,
                0.456,
                -0.45,
                1.251,
                1.272,
                -0.417,
                0.196,
                -0.092,
                0.476,
                1.461,
                1.032,
                1.59,
                0.929,
                0.371,
                0.476,
                1.184
                ]

print("Time: ", time, "Acceleration: ", acceleration)

velocity = cumtrapz(acceleration, time, initial=0)

displacement = cumtrapz(velocity, time, initial=0)

total_distance = cumtrapz(np.abs(velocity), time, initial=0)[-1]

plt.figure(figsize=(10, 6))

# Acceleration
plt.subplot(3, 1, 1)
plt.plot(time, acceleration, label='Acceleration')
plt.title('Acceleration vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration (m/s^2)')
plt.grid(True)

# Velocity
plt.subplot(3, 1, 2)
plt.plot(time, velocity, label='Velocity', color='g')
plt.title('Velocity vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.grid(True)

# Displacement
plt.subplot(3, 1, 3)
plt.plot(time, displacement, label='Displacement', color='r')
plt.title('Displacement vs Time')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (m)')
plt.grid(True)
print("Total Displacement: ", displacement[-1])

plt.tight_layout()
plt.show()
