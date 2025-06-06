import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math


def get_path(source, destination, angle):
    print(f"Source: {source}, Destination: {destination}")
    source = np.array(source)
    destination = np.array(destination)

    # Compute total distance
    direction = destination - source
    total_distance = np.linalg.norm(direction)
    print(f"Distance: {total_distance:.2f} cm")

    y = destination[1] - source[1]
    x = destination[0] - source[0]

    x1 = (math.cos(angle/57.2957795)*x)-(math.sin(angle/57.2957795)*y)
    y1 = (math.sin(angle/57.2957795)*x)+(math.cos(angle/57.2957795)*y)

    scale = 0.002
    y_scaled = y1 * total_distance * scale
    x_scaled = x1 * total_distance * scale

    # print(f"After Scale: {x_scaled},{y_scaled}")

    theta = np.arctan2(y_scaled, x_scaled)  # Angle in radians
    theta_deg = np.degrees(theta)  # Convert to degrees
    if theta_deg < 0:
        theta_deg = 360 - abs(theta_deg)
    print(f"Angle: {theta_deg:.2f}")

    limits = [6, 40, 100]

    x_scaled = round(x_scaled, 2)
    y_scaled = round(y_scaled, 2)
    print(f"Before Small: {x_scaled},{y_scaled}")

    if abs(x_scaled) < abs(y_scaled):
        if x_scaled != 0:
            term = x_scaled
        else:
            term = y_scaled
    elif abs(x_scaled) > abs(y_scaled):
        if y_scaled != 0:
            term = y_scaled
        else:
            term = x_scaled

    if total_distance < limits[2]:
        lim = limits[2]*(total_distance/200)
        limits[1] = lim if lim > limits[0] else limits[0]
    if abs(term) < limits[0]:
        x_scaled = math.ceil(round(x_scaled * (limits[0]/abs(term)), 2))
        y_scaled = math.ceil(round(y_scaled * (limits[0]/abs(term)), 2))

    print(f"After Small: {x_scaled},{y_scaled}")

    term = x_scaled if abs(x_scaled) > abs(y_scaled) else y_scaled

    if abs(term) > limits[1]:
        x_scaled = math.ceil(round(x_scaled * (limits[1]/abs(term)), 2))
        y_scaled = math.ceil(round(y_scaled * (limits[1]/abs(term)), 2))

    print(f"After Large: {x_scaled},{y_scaled}")

    theta = np.arctan2(y_scaled, x_scaled)  # Angle in radians
    theta_deg = np.degrees(theta)  # Convert to degrees
    if theta_deg < 0:
        theta_deg = 360 - abs(theta_deg)
    print(f"Angle: {theta_deg:.2f}")

    x_scaled = x_scaled if abs(x_scaled) >= limits[0] else 0
    y_scaled = y_scaled if abs(y_scaled) >= limits[0] else 0

    scaled_target = (round(x_scaled, 0), round(y_scaled, 0))
    print(f"Controller Replica: {scaled_target[0]}, {scaled_target[1]}\n")
    return scaled_target


# Initialize source and destination
source = [0, 0]
so = source.copy()
destination = [100, 50]
path_x = [source[0]]
path_y = [source[1]]

s = np.array(source)
d = np.array(destination)
direction = d - s
total_distance = np.linalg.norm(direction)

positions = [(source[0], source[1])]

# Track movement step by step
while total_distance > 3:
    st = get_path(source, destination, 0)
    source[0] = source[0] + (st[0] / 15)
    source[1] = source[1] + (st[1] / 15)
    s = np.array(source)
    direction = d - s
    total_distance = np.linalg.norm(direction)
    print("Distance Remaining:", total_distance)
    positions.append((source[0], source[1]))

if True:
    # Create animation
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(min(so[0], destination[0])-20,
                max(so[0], destination[0])+20)
    ax.set_ylim(min(so[1], destination[1])-20,
                max(so[1], destination[1])+20)
    ax.set_xlabel("X-coordinate")
    ax.set_ylabel("Y-coordinate")
    ax.set_title("Step-by-Step Path Visualization")
    ax.grid(True)

    # Plot start and destination points
    ax.scatter(so[0], so[1], color='g',
               marker='s', s=100, label="Start Point")
    ax.scatter(destination[0], destination[1], color='r',
               marker='X', s=100, label="Destination")
    ax.legend()

    line, = ax.plot([], [], marker='o', linestyle='-',
                    color='b', label="Movement Path")
    robot, = ax.plot([], [], marker='o', color='black', markersize=8)

    def init():
        line.set_data([], [])
        robot.set_data([], [])
        return line, robot

    def update(frame):
        x_vals = [p[0] for p in positions[:frame + 1]]
        y_vals = [p[1] for p in positions[:frame + 1]]
        line.set_data(x_vals, y_vals)
        robot.set_data(x_vals[-1], y_vals[-1])  # Current position of the robot
        return line, robot

    ani = animation.FuncAnimation(fig, update, frames=len(
        positions), init_func=init, blit=True, interval=30)

    plt.show()
