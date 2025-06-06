# This script was used to generate images for our presentation

import numpy as np
import cv2

def center_find(cnt, img):
    M = cv2.moments(cnt)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    
    # Draw a circle on the center of the contour and label it
    cv2.circle(img, (cX, cY), 3, (255, 255, 255), -1)
    cv2.putText(img, "center", (cX - 20, cY - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return img, cX, cY

# Load a color image
img = cv2.imread('OriginalImage.png')
ColorLower = (0, 139, 0)
ColorUpper = (180, 255, 255)

# Show the original image
cv2.imshow('image', img)
cv2.waitKey(0)

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Construct a mask for the specified color
mask = cv2.inRange(hsv, ColorLower, ColorUpper)
cv2.imshow('image', mask)
cv2.waitKey(0)

# Erode small contours
mask = cv2.erode(mask, None, iterations=5)
cv2.imshow('image', mask)
cv2.waitKey(0)

# Dilate remaining contours
mask = cv2.dilate(mask, None, iterations=5)
cv2.imshow('image', mask)
cv2.waitKey(0)

cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

contour_img = np.zeros_like(img)
for cnt in cnts:
    contour_img, _, _ = center_find(cnt, contour_img)

cv2.drawContours(contour_img, cnts, -1, (0, 255, 0), 3)
cv2.imshow('image', contour_img)
cv2.waitKey(0)  # Wait for a key press before closing windows
cv2.destroyAllWindows()
