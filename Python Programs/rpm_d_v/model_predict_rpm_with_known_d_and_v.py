import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import MinMaxScaler
from sklearn.linear_model import LinearRegression
import joblib


# Given data
voltage = np.array([
    24.6, 24.6, 24.6, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5,
    24.4, 24.4, 24.4, 24.3, 24.2, 24.2, 24.2, 24.1, 24.1, 24,
    24, 23.9, 23.9, 23.8
])

distance = np.array([
    1.9, 2.1, 2.27, 2.45, 2.6, 2.82, 2.93, 3.2, 3.4, 3.58,
    3.8, 4, 4.25, 4.5, 4.6, 4.8, 5.05, 5.15, 5.3, 5.7,
    5.9, 6.25, 6.45, 6.6
])

rpm = np.array([
    1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
    2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400,
    3500, 3600, 3700, 3800
])

# Scale data using MinMaxScaler
scaler = MinMaxScaler()
X = np.column_stack((distance, voltage))
X_scaled = scaler.fit_transform(X)

# Fit linear regression model
model = LinearRegression()
model.fit(X_scaled, rpm)

# Get equation coefficients
coefficients = model.coef_
intercept = model.intercept_
equation = f"RPM = {coefficients[0]:.5f} * Distance + {coefficients[1]:.5f} * Voltage + {intercept:.5f}"

# Print equation
print("Equation for RPM:")
print(equation)

# Save the model
joblib.dump(model, 'rpm_prediction_model.pkl')

# Sample input
sample_distance = 1.9
sample_voltage = 23.0
sample_input = scaler.transform([[sample_distance, sample_voltage]])
# Ensure RPM is non-negative
predicted_rpm = max(model.predict(sample_input)[0], 0)
joblib.dump(scaler, 'scaler.pkl')


print(
    f"Predicted RPM for Distance={sample_distance}m and Voltage={sample_voltage}V: {predicted_rpm:.2f}")

# Plot graph for different voltage levels
plt.figure(figsize=(8, 6))
voltage_levels = np.linspace(min(voltage), max(voltage), 5)
distance_range = np.linspace(min(distance), max(distance), 100)

for v in voltage_levels:
    rpm_predictions = [max(model.predict(scaler.transform([[d, v]]))[
                           0], 0) for d in distance_range]
    plt.plot(distance_range, rpm_predictions,
             linestyle='dashed', label=f'Voltage={v:.1f}V')

plt.scatter(distance, rpm, color='blue', label='Actual Data')
plt.xlabel('Distance (m)')
plt.ylabel('RPM')
plt.title('RPM vs Distance for Different Voltages')
plt.legend()
plt.grid()
plt.show()


# import numpy as np
# import matplotlib.pyplot as plt
# from sklearn.preprocessing import PolynomialFeatures
# from sklearn.linear_model import LinearRegression

# # Given data
# voltage = np.array([
#     24.6, 24.6, 24.6, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5,
#     24.4, 24.4, 24.4, 24.3, 24.2, 24.2, 24.2, 24.1, 24.1, 24,
#     24, 23.9, 23.9, 23.8
# ])

# distance = np.array([
#     1.9, 2.1, 2.27, 2.45, 2.6, 2.82, 2.93, 3.2, 3.4, 3.58,
#     3.8, 4, 4.25, 4.5, 4.6, 4.8, 5.05, 5.15, 5.3, 5.7,
#     5.9, 6.25, 6.45, 6.6
# ])

# rpm = np.array([
#     1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
#     2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400,
#     3500, 3600, 3700, 3800
# ])

# # Prepare input data
# X = np.column_stack((distance, voltage))
# poly = PolynomialFeatures(degree=2, include_bias=False)
# X_poly = poly.fit_transform(X)

# # Fit polynomial regression model
# model = LinearRegression()
# model.fit(X_poly, rpm)

# # Get equation coefficients
# coefficients = model.coef_
# intercept = model.intercept_

# # Generate equation string
# feature_names = poly.get_feature_names_out(["Distance", "Voltage"])
# equation = "RPM = " + \
#     " + ".join([f"{coeff:.5f} * {name}" for coeff,
#                name in zip(coefficients, feature_names)])
# equation += f" + {intercept:.5f}"

# # Print equation
# print("Equation for RPM:")
# print(equation)


# # Plot graph
# distance_range = np.linspace(0, 10, 1000)
# voltage_fixed = 24.6  # Example fixed voltage
# rpm_predictions = [model.predict(poly.transform([[d, voltage_fixed]]))[
#     0] for d in distance_range]

# plt.figure(figsize=(8, 6))
# plt.scatter(distance, rpm, color='blue', label='Actual Data')
# plt.plot(distance_range, rpm_predictions, color='red',
#          linestyle='dashed', label=f'Predicted (Voltage={voltage_fixed}V)')
# plt.xlabel('Distance (m)')
# plt.ylabel('RPM')
# plt.title('RPM vs Distance with Fixed Voltage')
# plt.legend()
# plt.grid()
# plt.show()

# while True:
#     # Sample input
#     sample_distance = float(input("Enter Distance(m): "))
#     sample_voltage = float(input("Enter Voltage(v): "))
#     sample_input = np.array([[sample_distance, sample_voltage]])
#     sample_input_poly = poly.transform(sample_input)
#     predicted_rpm = model.predict(sample_input_poly)[0]

#     print(
#         f"Predicted RPM for Distance={sample_distance}m and Voltage={sample_voltage}V: {predicted_rpm:.2f}")


# # import numpy as np
# # import pandas as pd
# # import matplotlib.pyplot as plt
# # from sklearn.linear_model import LinearRegression
# # from sklearn.preprocessing import PolynomialFeatures
# # from sklearn.pipeline import make_pipeline

# # # Given data
# # voltage = np.array([
# #     24.6, 24.6, 24.6, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5,
# #     24.4, 24.4, 24.4, 24.3, 24.2, 24.2, 24.2, 24.1, 24.1, 24,
# #     24, 23.9, 23.9, 23.8
# # ])

# # distance = np.array([
# #     1.9, 2.1, 2.27, 2.45, 2.6, 2.82, 2.93, 3.2, 3.4, 3.58,
# #     3.8, 4, 4.25, 4.5, 4.6, 4.8, 5.05, 5.15, 5.3, 5.7,
# #     5.9, 6.25, 6.45, 6.6
# # ])

# # rpm = np.array([
# #     1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
# #     2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400,
# #     3500, 3600, 3700, 3800
# # ])

# # # Reshaping data
# # X = np.column_stack((distance, voltage))
# # y = rpm

# # # Create a polynomial regression model
# # poly_degree = 2  # Adjust the degree as needed
# # model = make_pipeline(PolynomialFeatures(
# #     degree=poly_degree), LinearRegression())
# # model.fit(X, y)


# # def predict_rpm(distance_value, voltage_value):
# #     return model.predict(np.array([[distance_value, voltage_value]]))[0]


# # # Example usage
# # distance_input = 5.0
# # voltage_input = 24.0
# # predicted_rpm = predict_rpm(distance_input, voltage_input)
# # print(
# #     f"Predicted RPM for Distance={distance_input}m and Voltage={voltage_input}V: {predicted_rpm:.2f}")

# # # Plot the graph
# # plt.figure(figsize=(8, 6))
# # plt.scatter(distance, rpm, color='blue', label='Actual Data')

# # distance_range = np.linspace(min(distance), max(distance), 100)
# # rpm_predictions = [predict_rpm(d, 24.0) for d in distance_range]
# # plt.plot(distance_range, rpm_predictions, color='red',
# #          label='Fitted Curve (Voltage=24.0V)')

# # plt.xlabel("Distance (m)")
# # plt.ylabel("RPM")
# # plt.title("RPM vs Distance with Voltage Considered")
# # plt.legend()
# # plt.grid()
# # plt.show()


# import numpy as np
# import pandas as pd
# import matplotlib.pyplot as plt
# from sklearn.linear_model import LinearRegression
# from sklearn.preprocessing import PolynomialFeatures
# from sklearn.pipeline import make_pipeline

# # Given data
# voltage = np.array([
#     24.6, 24.6, 24.6, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5, 24.5,
#     24.4, 24.4, 24.4, 24.3, 24.2, 24.2, 24.2, 24.1, 24.1, 24,
#     24, 23.9, 23.9, 23.8
# ])

# distance = np.array([
#     1.9, 2.1, 2.27, 2.45, 2.6, 2.82, 2.93, 3.2, 3.4, 3.58,
#     3.8, 4, 4.25, 4.5, 4.6, 4.8, 5.05, 5.15, 5.3, 5.7,
#     5.9, 6.25, 6.45, 6.6
# ])

# rpm = np.array([
#     1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300, 2400,
#     2500, 2600, 2700, 2800, 2900, 3000, 3100, 3200, 3300, 3400,
#     3500, 3600, 3700, 3800
# ])

# # Reshaping data
# X = np.column_stack((distance, voltage))
# y = rpm

# # Create a polynomial regression model
# poly_degree = 2  # Adjust the degree as needed
# model = make_pipeline(PolynomialFeatures(
#     degree=poly_degree), LinearRegression())
# model.fit(X, y)


# def predict_rpm(distance_value, voltage_value):
#     return model.predict(np.array([[distance_value, voltage_value]]))[0]


# # Example usage
# distance_input = 2.6
# voltage_input = 23
# predicted_rpm = predict_rpm(distance_input, voltage_input)
# print(
#     f"Predicted RPM for Distance={distance_input}m and Voltage={voltage_input}V: {predicted_rpm:.2f}")

# # Plot the graph
# plt.figure(figsize=(8, 6))
# plt.scatter(distance, rpm, color='blue', label='Actual Data')

# distance_range = np.linspace(min(distance), max(distance), 100)
# voltage_range = np.linspace(max(voltage), min(voltage), 100)
# rpm_predictions = [predict_rpm(d, v)
#                    for d, v in zip(distance_range, voltage_range)]
# # plt.plot(distance_range, rpm_predictions, color='red',
# #          label='Adjusted RPM considering Voltage Drop')

# # plt.xlabel("Distance (m)")
# # plt.ylabel("RPM")
# # plt.title("RPM vs Distance with Voltage Drop Considered")
# # plt.legend()
# # plt.grid()
# # plt.show()
