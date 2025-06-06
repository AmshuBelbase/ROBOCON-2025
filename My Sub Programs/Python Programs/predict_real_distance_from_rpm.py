from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import numpy as np
import matplotlib.pyplot as plt
# import serial

# serial_port = '/dev/ttyACM0'
# baud_rate = 115200
# ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Given data
# Equation: D = -0.1044 + (0.001282 * R) + (0.000000155 * R^2)
# Equation: D = -1.6353 + (0.002769 * R) + (-0.000000190 * R^2)

# rpm_real = np.array([1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
#                      2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500])
# distance_real = np.array([2.12, 2.3, 2.52, 2.73, 2.92, 3.06, 3.34, 3.52, 3.68, 3.83,
#                           4.1, 4.35, 4.52, 4.72, 4.9, 5.09, 5.18, 5.42, 5.86, 6, 6.5])


# Equation: D = -1.6353 + (0.002769 * R) + (-0.000000190 * R^2)

# rpm_real = np.array([1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
#                      2500, 2600, 3000])
# distance_real = np.array(
#     [2.1, 2.3, 2.52, 2.75, 2.95, 3.12, 3.35, 3.48, 3.73, 3.9, 4.2, 4.25, 4.95])

# rpm_real = np.array([1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400, 2500,
#                     2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500, 3600, 3700, 3800])

# distance_real = np.array(
#     [1.9, 2.1, 2.27, 2.45, 2.6, 2.82, 2.93, 3.2, 3.4, 3.58, 3.8, 4, 4.25, 4.5, 4.6, 4.8, 5.05, 5.15, 5.3, 5.7, 5.9, 6.25, 6.45, 6.6])


# rpm_real = np.array([2000, 2200, 2400, 2600, 2800, 3000,
#                     3200, 3400, 3600, 3800, 3900, 4000, 4100, 4200])

# distance_real = np.array(
#     [2.8, 3.25, 3.65, 4.1, 4.5, 4.9, 5.3, 5.8, 6.1, 6.7, 6.9, 7.3, 7.4, 8])

rpm_real = np.array([1500, 1500, 1500, 1600, 1600, 1600, 1700, 1700, 1700, 1800, 1800, 1800, 1900, 1900, 1900, 2000, 2000, 2000,
                     2000, 2100, 2100, 2100, 2200, 2200, 2200, 2200, 2300, 2300, 2300, 2400, 2400, 2400, 2400, 2500, 2500, 2500, 2600, 2600, 2600, 2600, 2700, 2700, 2800, 2800, 2800, 2900, 2900, 3000, 3000, 3000, 3000, 3100, 3100,                  3200, 3200, 3200, 3300, 3300, 3400, 3400, 3400, 3500, 3500, 3600, 3600, 3700, 3800, 3800, 3900, 4000, 4100, 4200])


distance_real = np.array([2.12, 2.1, 1.9, 2.3, 2.3, 2.1, 2.52, 2.52, 2.27, 2.73, 2.75, 2.45, 2.92, 2.95, 2.6, 3.06, 3.12, 2.82, 2.8, 3.34, 3.35, 2.93, 3.52, 3.48, 3.2, 3.25, 3.68, 3.73, 3.4, 3.83, 3.9, 3.58,
                         3.65, 4.1, 4.2, 3.8, 4.35, 4.25, 4, 4.1, 4.52, 4.25, 4.72, 4.5, 4.5, 4.9, 4.6, 5.09, 4.95, 4.8, 4.9, 5.18, 5.05, 5.42, 5.15, 5.3, 5.86, 5.3, 6, 5.7, 5.8, 6.5, 5.9, 6.25, 6.1, 6.45, 6.6, 6.7, 6.9, 7.3, 7.4, 8])

# Polynomial Regression of Degree 2
poly = PolynomialFeatures(degree=2)
X_poly = poly.fit_transform(distance_real.reshape(-1, 1))

# Train the model
model = LinearRegression()
model.fit(X_poly, rpm_real)
predictions = model.predict(X_poly)

# Plot results
plt.figure(figsize=(8, 5))
plt.scatter(distance_real, rpm_real, color='blue', label='Real Data')
plt.plot(distance_real, predictions, color='red',
         label='Polynomial Fit (Degree 2)')
plt.xlabel("RPM")
plt.ylabel("Real Distance (m)")
plt.title("Polynomial Regression (Degree 2)")
plt.legend()
plt.grid()
plt.show()

# Print regression equation coefficients
print(
    f"Equation: R = {model.intercept_:.4f} + ({model.coef_[1]:.6f} * D) + ({model.coef_[2]:.9f} * D^2)")


# Function to Predict Distance Based on User Input
def predict_distance():
    try:
        d_input = float(input("Enter the distance value: ")
                        )  # Take user input
        d_transformed = poly.transform(
            np.array([[d_input]]))  # Transform input
        predicted_rpm = model.predict(d_transformed)[0]  # Predict
        print(f"Predicted RPM: {predicted_rpm:.4f} meters")
        # Convert to bytes
        # data = (str(int(predicted_rpm))) + "\n"
        # ser.write(data.encode())
    except ValueError:
        print("Invalid input. Please enter a numerical value.")


while True:
    # Call the function to predict distance
    predict_distance()
