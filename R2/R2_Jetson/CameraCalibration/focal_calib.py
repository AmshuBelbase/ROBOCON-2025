import numpy as np

data = np.load('MultiMatrixzed21080.npz')  # Replace with your actual filename
print(data.files)
camera_matrix = data['camMatrix']
print("Camera Matrix:\n", camera_matrix)
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
print("Focal length (fx):", fx)
print("Focal length (fy):", fy)
focal_length = (fx + fy) / 2
print("Average focal length:", focal_length)
