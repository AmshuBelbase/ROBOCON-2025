import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.linear_model import LinearRegression

# # Data
xu = np.array([21.5, 25, 28, 31, 34, 37, 40, 43, 47, 50, 53, 56,
               59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95])
yu = np.array([311, 356, 400, 457, 497, 546, 592, 635, 693, 735, 792, 833,
               870, 920, 960, 1000, 1050, 1098, 1138, 1185, 1225, 1272, 1325, 1365, 1403])
zu = np.array([24.8, 24.8, 24.8, 24.8, 24.8, 24.8, 24.8, 24.7, 24.7, 24.7,
               24.7, 24.7, 24.7, 24.6, 24.6, 24.6, 24.6, 24.6, 24.6, 24.6,
               24.5, 24.5, 24.5, 24.5, 24.5])

# Create the regression model
X = np.column_stack((yu, zu))
model = LinearRegression().fit(X, xu)

# Define a function to predict x given y and z


def predict_x(y, z):
    return model.predict(np.array([[y, z]]))[0]


# Generate predictions for plotting
y_pred = np.linspace(yu.min(), yu.max(), 50)
z_pred = np.linspace(zu.min(), zu.max(), 50)
Y_pred, Z_pred = np.meshgrid(y_pred, z_pred)
X_pred = model.predict(np.column_stack(
    (Y_pred.ravel(), Z_pred.ravel()))).reshape(Y_pred.shape)

# Plotting
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(yu, zu, xu, color='blue', label='Original Data')
ax.plot_surface(Y_pred, Z_pred, X_pred, color='pink',
                alpha=0.5, rstride=100, cstride=100)

ax.set_xlabel('RPM values')
ax.set_ylabel('Battery Voltage values')
ax.set_zlabel('Predicted Duty Cycle values')
ax.legend()
plt.show()

# import numpy as np
# from sklearn.linear_model import LinearRegression

# # Data
# xu = np.array([21.5, 25, 28, 31, 34, 37, 40, 43, 47, 50, 53, 56,
#                59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95])
# yu = np.array([311, 356, 400, 457, 497, 546, 592, 635, 693, 735, 792, 833,
#                870, 920, 960, 1000, 1050, 1098, 1138, 1185, 1225, 1272, 1325, 1365, 1403])
# zu = np.array([24.8, 24.8, 24.8, 24.8, 24.8, 24.8, 24.8, 24.7, 24.7, 24.7,
#                24.7, 24.7, 24.7, 24.6, 24.6, 24.6, 24.6, 24.6, 24.5, 24.5,
#                24.4, 24.4, 24.4, 24.4, 24.4])

# # Create the regression model
# X = np.column_stack((yu, zu))
# model = LinearRegression().fit(X, xu)

# # Get the coefficients
# a, b = model.coef_
# c = model.intercept_

# # Display the equation
# print(f"The equation is: x = {a:.4f} * y + {b:.4f} * z + {c:.4f}")

# # Function to calculate x given y and z


# def predict_x(y, z):
#     return a * y + b * z + c


# while True:
#     y = float(input("RPM:"))
#     z = float(input("Battery:"))
#     print("Required Duty Cycle:", predict_x(y, z))
