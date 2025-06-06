from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import numpy as np
import matplotlib.pyplot as plt
import serial
import time


def connect_to_teensy():
    try:
        ser = serial.Serial('COM3', 9600, timeout=1,
                            rtscts=False, dsrdtr=False)
        print("Connected to Arduino.")
        time.sleep(0.5)
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")
        exit()

# Given data

# rpm_real = np.array([1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
#                      2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400, 3500])
# distance_real = np.array([2.12, 2.3, 2.52, 2.73, 2.92, 3.06, 3.34, 3.52, 3.68, 3.83,
#                           4.1, 4.35, 4.52, 4.72, 4.9, 5.09, 5.18, 5.42, 5.86, 6, 6.5])


rpm_real = np.array([1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
                     2500, 2600, 3000])
distance_real = np.array(
    [2.1, 2.3, 2.52, 2.75, 2.95, 3.12, 3.35, 3.48, 3.73, 3.9, 4.2, 4.25, 4.95])


# Polynomial Regression of Degree 2
'''poly = PolynomialFeatures(degree=2)
X_poly = poly.fit_transform(rpm_real.reshape(-1, 1))
'''
poly = PolynomialFeatures(degree=2)
X_poly = poly.fit_transform(distance_real.reshape(-1, 1))

# Train the model
model = LinearRegression()
'''model.fit(X_poly, distance_real)
predictions = model.predict(X_poly)'''
model.fit(X_poly, rpm_real)
predictions = model.predict(X_poly)

# # Plot results
# plt.figure(figsize=(8, 5))
# plt.scatter(distance_real, rpm_real, color='blue', label='Real Data')
# plt.plot(distance_real, predictions, color='red',
#          label='Polynomial Fit (Degree 2)')
# plt.ylabel("RPM")
# plt.xlabel("Real Distance (m)")
# plt.title("Polynomial Regression (Degree 2)")
# plt.legend()
# plt.grid()
# plt.show()

# Print regression equation coefficients
print(
    f"Equation: rpm = {model.intercept_:.4f} + ({model.coef_[1]:.6f} * d) + ({model.coef_[2]:.9f} * d^2)")


def send_rpm(ser):
    try:
        while True:
            distance = float(input("D: "))
            rpm_calc = (model.intercept_) + \
                (model.coef_[1]*distance)+(model.coef_[2]*distance*distance)
            message = f"{rpm_calc}\n"
            print(message)
            ser.write(message.encode('utf-8'))
            ser.flush()
            time.sleep(1)
            # while True:
            if ser.in_waiting > 0:
                data_from_pico = ser.readline().strip().decode()
                print(f"Received from Teensy: {data_from_pico}")
            else:
                print("..")
    finally:
        ser.close()
        print("Serial connection closed.")


ser = connect_to_teensy()
send_rpm(ser)
