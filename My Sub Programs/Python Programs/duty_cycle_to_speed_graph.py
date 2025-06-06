import numpy as np
import matplotlib.pyplot as plt

xu = np.array([21.5, 25, 28, 31, 34, 37, 40, 43, 47, 50, 53, 56,
              59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95])
yu = np.array([311, 356, 400, 457, 497, 546, 592, 635, 693, 735, 792, 833, 870, 920, 960, 1000, 1050, 1098, 1138,
               1185, 1225, 1272, 1325, 1365, 1403])

xd = np.array([22, 25, 28, 31, 34, 37, 40, 43, 46, 49, 52, 55,
              58, 61, 64, 67, 70, 73, 76, 79, 82, 85, 88, 91, 94.5])
yd = np.array([300, 338, 396, 435, 480, 527, 565, 615, 657, 704, 748, 790,
              840, 880, 923, 972, 1018, 1064, 1103, 1157, 1193, 1233, 1280, 1320, 1370])

degree = 14


def get_graph(x, y, sc, pl, title=""):
    coefficients = np.polyfit(x, y, degree)

    polynomial = np.poly1d(coefficients)
    x_values = np.linspace(min(x), max(x), 100)
    y_values = polynomial(x_values)

    plt.scatter(x, y, color=sc, label='Data Points')
    plt.plot(x_values, y_values, color=pl,
             label=f'Polynomial Fit (degree {degree})')
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(title)

    return polynomial


# Plot the first graph in a new figure
plt.figure("dc vs speed")  # Naming the window for clarity
d2s_up = get_graph(xu, yu, "red", "blue", "dc vs speed")
d2s_dn = get_graph(xd, yd, "green", "black", "dc vs speed")
plt.show(block=False)

# Plot the second graph in another new figure
plt.figure("speed vs dc")
s2d_up = get_graph(yu, xu, "red", "blue", "speed vs dc")
s2d_dn = get_graph(yd, xd, "green", "black", "speed vs dc")
plt.show(block=False)

plt.pause(0.1)

while True:
    s = input("Enter either duty cycle or speed: ")
    x = float(s[1:])
    if s[0] == 'u':
        if (x > 100):
            speed = x
            dc = s2d_up(speed)
            print("Duty Cycle: ", dc)
        else:
            dc = x
            speed = d2s_up(dc)
            print("Speed:", speed)
    elif s[0] == 'd':
        if (x > 100):
            speed = x
            dc = s2d_dn(speed)
            print("Duty Cycle: ", dc)
        else:
            dc = x
            speed = d2s_dn(dc)
            print("Speed:", speed)
