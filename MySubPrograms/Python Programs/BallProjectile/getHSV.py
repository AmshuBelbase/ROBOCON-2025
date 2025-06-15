import cv2 as cv
import numpy as np


def nothing(x):
    pass


# Load the image
image_path = 'Untitled.png'  # Replace with your image file path
image = cv.imread(image_path)

if image is None:
    print("Error: Could not load image.")
    exit()

# Convert the image to HSV
hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

# Create a window with trackbars to adjust HSV values
cv.namedWindow("HSV Adjustments")
cv.createTrackbar("Lower H", "HSV Adjustments", 0, 179, nothing)
cv.createTrackbar("Lower S", "HSV Adjustments", 0, 255, nothing)
cv.createTrackbar("Lower V", "HSV Adjustments", 0, 255, nothing)
cv.createTrackbar("Upper H", "HSV Adjustments", 179, 179, nothing)
cv.createTrackbar("Upper S", "HSV Adjustments", 255, 255, nothing)
cv.createTrackbar("Upper V", "HSV Adjustments", 255, 255, nothing)

while True:
    # Get the current positions of the trackbars for lower and upper HSV
    lower_h = cv.getTrackbarPos("Lower H", "HSV Adjustments")
    lower_s = cv.getTrackbarPos("Lower S", "HSV Adjustments")
    lower_v = cv.getTrackbarPos("Lower V", "HSV Adjustments")
    upper_h = cv.getTrackbarPos("Upper H", "HSV Adjustments")
    upper_s = cv.getTrackbarPos("Upper S", "HSV Adjustments")
    upper_v = cv.getTrackbarPos("Upper V", "HSV Adjustments")

    # Set lower and upper bounds for HSV
    lower_hsv = np.array([lower_h, lower_s, lower_v])
    upper_hsv = np.array([upper_h, upper_s, upper_v])

    # Create a mask with the specified HSV range
    mask = cv.inRange(hsv_image, lower_hsv, upper_hsv)
    result = cv.bitwise_and(image, image, mask=mask)

    # Show the original, mask, and result
    cv.imshow("Original Image", image)
    cv.imshow("Mask", mask)
    cv.imshow("Result", result)

    # Break the loop on 'q' key press
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Print the final HSV values
print("Lower HSV:", lower_hsv)
print("Upper HSV:", upper_hsv)

cv.destroyAllWindows()
