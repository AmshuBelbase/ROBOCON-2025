from sklearn.linear_model import LinearRegression
import numpy as np

xu = [21.5, 25, 28, 31, 34, 37, 40, 43, 47, 50, 53, 56,
      59, 62, 65, 68, 71, 74, 77, 80, 83, 86, 89, 92, 95]
yu = [311, 356, 400, 457, 497, 546, 592, 635, 693, 735, 792, 833, 870, 920, 960, 1000, 1050, 1098, 1138,
      1185, 1225, 1272, 1325, 1365, 1403]

X = np.array(xu).reshape(-1, 1)  # put your duty cycle in here
y = yu  # put your rpm's in here

model = LinearRegression()
model.fit(X, y)

# put the duty cycle of which you want to predict rpm here
X_predict = [[50], [53], [56], [59]]
y_predict = model.predict(X_predict)

print(y_predict)
