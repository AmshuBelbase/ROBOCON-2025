import numpy as np
import matplotlib.pyplot as plt


def divide_path_step_cm(source, destination, steps):
    global xn, yn
    source = np.array(source)
    destination = np.array(destination)

    # Compute total distance
    direction = destination - source
    total_distance = np.linalg.norm(direction)
    print(total_distance)
    if total_distance == 0:  # If source and destination are the same
        xn = 0
        yn = 0
        angle = 0

        data = f"{xn},{yn},{angle}\n"
        # ser.write(data.encode())
        print(f"Sent: {data.strip()}")

        return

    # Normalize direction vector
    unit_direction = direction / total_distance

    # Generate points at step intervals
    points = [tuple(source)]
    for d in np.arange(steps, total_distance, steps):
        new_point = source + unit_direction * d
        points.append(tuple(new_point))

    # Append destination if remaining distance is less than 2 cm
    if total_distance % 2 != 0:
        points.append(tuple(destination))

    lxy = points[1]

    print(source[0], source[1])
    print(points)
    print(lxy[0], lxy[1])

    xn = lxy[0]
    yn = lxy[1]

    xn -= source[0]
    yn -= source[1]

    print(xn, yn)

    xn = int(xn*4)
    yn = int(yn*4)

    print(xn, yn)

    angle = 0

    data = f"{xn},{yn},{angle}\n"
    # ser.write(data.encode())
    print(f"Sent: {data.strip()}")

    return points


def visualize_path(source, destination, steps):
    path_points = divide_path_step_cm(source, destination, steps)

    x_values, y_values = zip(*path_points)

    plt.figure(figsize=(6, 6))
    plt.plot([source[0], destination[0]], [
             source[1], destination[1]], 'b-', label='Path')
    plt.scatter(x_values, y_values, color='red', label='Points', zorder=3)
    plt.scatter(*source, color='green', marker='o', label='Source', zorder=4)
    plt.scatter(*destination, color='blue', marker='x',
                label='Destination', zorder=4)

    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Path Visualization')
    plt.legend()
    plt.grid()
    plt.show()


# Example usage
source = (0, 0)
destination = (100, 0)
steps = 10
visualize_path(source, destination, steps)


# import numpy as np
# import matplotlib.pyplot as plt


# def bezier_curve(xc, yc, xf, yf, steps=100, curvature=100):
#     # Define a control point to make a smooth curve
#     control_x = (xc + xf) / 2  # Midpoint in X direction
#     control_y = min(yc, yf) - abs(xf - xc) / curvature  # Adjust for curvature

#     t = np.linspace(0, 1, steps)

#     # Quadratic Bézier curve equation
#     x = (1 - t) ** 2 * xc + 2 * (1 - t) * t * control_x + t ** 2 * xf
#     y = (1 - t) ** 2 * yc + 2 * (1 - t) * t * control_y + t ** 2 * yf

#     return x, y


# def plot_path(xc, yc, xf, yf):
#     x, y = bezier_curve(xc, yc, xf, yf, curvature=100)

#     plt.plot(x, y, label="Bezier Path")
#     plt.scatter([xc, xf], [yc, yf], color='red', label='Start/End')
#     plt.legend()
#     plt.xlabel("X")
#     plt.ylabel("Y")
#     plt.title("Smooth Curve Path from (xc, yc) to (xf, yf)")
#     plt.grid()
#     plt.show()


# # Example usage
# xc, yc = 0, 0   # Start position
# xf, yf = 100, 0  # End position
# plot_path(xc, yc, xf, yf)


# import heapq
# import matplotlib.pyplot as plt


# def heuristic(x1, y1, x2, y2):
#     """Calculate the heuristic (Euclidean distance)."""
#     return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


# def get_neighbors(x, y):
#     """Returns all possible 8-directional moves."""
#     return [
#         (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
#         (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)
#     ]


# def a_star(xc, yc, xf, yf):
#     """Finds the shortest path using A* algorithm."""
#     open_set = []
#     heapq.heappush(open_set, (0, xc, yc))
#     came_from = {}
#     g_score = {(xc, yc): 0}
#     f_score = {(xc, yc): heuristic(xc, yc, xf, yf)}

#     while open_set:
#         _, x, y = heapq.heappop(open_set)

#         if (x, y) == (xf, yf):
#             path = []
#             while (x, y) in came_from:
#                 path.append((x, y))
#                 x, y = came_from[(x, y)]
#             path.append((xc, yc))
#             return path[::-1]

#         for nx, ny in get_neighbors(x, y):
#             tentative_g_score = g_score[(x, y)] + heuristic(x, y, nx, ny)
#             if (nx, ny) not in g_score or tentative_g_score < g_score[(nx, ny)]:
#                 came_from[(nx, ny)] = (x, y)
#                 g_score[(nx, ny)] = tentative_g_score
#                 f_score[(nx, ny)] = tentative_g_score + \
#                     heuristic(nx, ny, xf, yf)
#                 heapq.heappush(open_set, (f_score[(nx, ny)], nx, ny))

#     return None  # No path found


# def visualize_path(path):
#     """Visualizes the path on a grid."""
#     if path is None:
#         print("No path found.")
#         return

#     x_vals, y_vals = zip(*path)
#     plt.plot(x_vals, y_vals, marker='o', linestyle='-')
#     plt.scatter([x_vals[0]], [y_vals[0]], color='green', label='Start')
#     plt.scatter([x_vals[-1]], [y_vals[-1]], color='red', label='Goal')
#     plt.legend()
#     plt.grid()
#     plt.show()


# # Example usage:
# xc, yc = 0, 0  # Current position
# xf, yf = 5, 8  # Target position
# path = a_star(xc, yc, xf, yf)
# print("Shortest Path:", path)
# visualize_path(path)
