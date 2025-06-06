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


# to get eqn to enter speed and get duty cycle
x = xu
y = yu

coefficients = np.polyfit(x, y, degree)
print(coefficients)

polynomial_dtos = np.poly1d(coefficients)
print("Polynomial equation to enter duty cycle and get speed:")
print(polynomial_dtos)


# to get eqn to enter speed and get duty cycle
x = yu
y = xu

coefficients = np.polyfit(x, y, degree)

polynomial_stod = np.poly1d(coefficients)
print("Polynomial equation to enter speed and get duty cycle:")
print(polynomial_stod)

# plotting

x_values = np.linspace(min(x), max(x), 100)
y_values = polynomial_stod(x_values)

print(x_values)
print(y_values)

plt.scatter(x, y, color='red', label='Data Points')
plt.plot(x_values, y_values, color='blue',
         label=f'Polynomial Fit (degree {degree})')
plt.legend()
plt.xlabel("x")
plt.ylabel("y")
plt.title("Polynomial Fit")
plt.show()

while True:

    x = float(input("Enter either duty cycle or speed: "))

    print(type(x))

    if (x > 100):

        # assume dutycycle

        speed = x
        dc = polynomial_stod(speed)
        print("Duty Cycle: ", dc)

    else:
        # assume speed

        dc = x
        speed = polynomial_dtos(dc)
        print("Speed:", speed)
