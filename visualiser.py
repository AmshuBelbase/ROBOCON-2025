import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Set up the serial connection (change COM port as needed)
ser = serial.Serial('COM14', 9600)  

x_data, y_data = [], []

# Initialize plot with fixed axes2a
fig, ax = plt.subplots()
ax.set_xlim(-400, 400)  # Fixed range for X (-400 to 400 cm)
ax.set_ylim(-400, 400)  # Fixed range for Y (-400 to 400 cm)
ax.set_xlabel("X Position (cm)")
ax.set_ylabel("Y Position (cm)")
ax.set_title("Real-Time Odometry Visualization")
ax.grid(True)  # Add a grid for better visualization
line, = ax.plot([], [], 'bo-', markersize=3)  # Blue dots for path

# Function to update the plot
def update(frame):
    if ser.in_waiting > 0:
        try:
            data = ser.readline().decode().strip()  # Read and decode data
            x, y = map(float, data.split(','))  # Convert to float
            x_data.append(x)
            y_data.append(y)

            # Update plot (without changing limits)
            line.set_data(x_data, y_data)
        except:
            pass  # Ignore errors due to bad formatting

    return line,

# Animate plot
ani = animation.FuncAnimation(fig, update, interval=100)

plt.show()

# Close serial connection on exit
ser.close()
