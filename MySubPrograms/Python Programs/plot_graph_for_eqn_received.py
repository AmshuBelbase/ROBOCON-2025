import numpy as np
import matplotlib.pyplot as plt

# Define the range of R values
D = np.linspace(1.5, 7, 2000)  # R ranges from 0 to 10000

# Define the first equation
# D1 = -0.1044 + (0.001282 * R) + (0.000000155 * R**2)

# Define the second equation
# D2 = -1.6353 + (0.002769 * R) + (-0.000000190 * R**2)

R1 = 176.8937 + (654.751174 * D) + (-20.378204632 * D ** 2)
R2 = 681.6445 + (338.915765 * D) + (25.608421688 * D ** 2)
R3 = 365.1831 + (627.600129 * D) + (-16.620472166 * D ** 2)
R4 = 272.4869 + (658.044224 * D) + (-19.914764905 * D ** 2)
R = 299.5785 + (613.746104 * D) + (-14.230117511 * D**2)

# Plot the graphs
plt.figure(figsize=(10, 6))
plt.plot(D, R1, label='R1 = 176.8937 + (654.751174 * D) + (-20.378204632 * D ** 2)',
         color='blue', linestyle="dashed")
# plt.plot(D, R2, label='R2 = 681.6445 + (338.915765 * D) + (25.608421688 * D ** 2)', color='red')
plt.plot(D, R3, label='R2 = 365.1831 + (627.600129 * D) + (-16.620472166 * D ** 2)',
         color='green', linestyle="dashed")
plt.plot(D, R4, label='R3 = 272.4869 + (658.044224 * D) + (-19.914764905 * D ** 2)',
         color='purple', linestyle="dashed")
plt.plot(D, R, label='R = 299.5785 + (613.746104 * D) + (-14.230117511 * D**2)',
         color='black')


# Add labels and title
plt.xlabel('RPM')
plt.ylabel('Distance')
plt.title('Graph: Distance vs RPM')
plt.legend()
plt.grid(True)

# Show the plot
plt.show()
