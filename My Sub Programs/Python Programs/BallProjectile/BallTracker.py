import math
import cv2 as cv
import numpy as np
import time


def color_masking(frame, color_lower, color_upper):
    # Convert frame to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Construct a mask for the specified color
    mask = cv.inRange(hsv, color_lower, color_upper)
    # Erode small contours
    mask = cv.erode(mask, None, iterations=5)
    # Dilate remaining contours
    mask = cv.dilate(mask, None, iterations=5)
    return mask


def large_contour_find(frame, mask):
    # Contour find function, draws contours and plots the center
    try:
        # Find contours on the mask
        cnts, _ = cv.findContours(
            mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # Check if contours are present
        if cnts:
            # Find largest contour
            largest_contour = max(cnts, key=cv.contourArea)
            # Create a blank image to draw contours on
            contour_img = np.zeros_like(frame)
            # Draw contours on the blank image
            cv.drawContours(contour_img, [largest_contour], -1, (0, 255, 0), 3)
            # Add the contours to the masked frame
            return [largest_contour, contour_img]
        else:
            return [None, mask]
    except:
        return [None, mask]


def center_find(cnt, img):
    # Iterate through countour and find center and plot on image
    M = cv.moments(cnt)
    m00 = M["m00"]
    if m00 != 0:
        cX = int(M["m10"] / m00)
        cY = int(M["m01"] / m00)
        # Draw a circle on center of contour and label it
        cv.circle(img, (cX, cY), 7, (255, 255, 255), -1)
        cv.putText(img, "center", (cX - 20, cY - 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return [img, cX, cY]
    else:
        return [img, None, None]


def velo_draw(c_data, img, fps):
    dt = 1 / fps  # Calculate Time step based on framerate
    vX = (c_data[-1][0] - c_data[-2][0]) / (dt)
    vY = (c_data[-1][1] - c_data[-2][1]) / (dt)
    aY = None
    if len(c_data) > 3:
        K = -(-9.81 * dt) / (vY - c_data[-2][4])
        aY = K * (vY - c_data[-2][4]) / dt
        c_data[-1][6] = K
        cv.putText(img, "Velocity: [" + "{:.2f}".format(-vY * K) + "]", (30, 40), cv.FONT_HERSHEY_SIMPLEX, 1,
                   (255, 255, 255), 2)
        time.sleep(.1)
        # scale  previous velocities
        if len(c_data) > 4:
            c_data[-2][3] = c_data[-2][6] * c_data[-2][3]
            c_data[-2][4] = c_data[-2][6] * c_data[-2][4]

    c_data[-1][3] = vX
    c_data[-1][4] = vY
    c_data[-1][5] = aY

    # print the velocity onto top corner

    return [c_data, img]


def traj_draw(c_data, contour_img):
    for index, data in enumerate(c_data):
        cv.circle(contour_img, [data[0], data[1]], 5, (0, 255, 0), -1)
        if index > 0:
            cv.line(contour_img, [data[0], data[1]], [
                    c_data[index - 1][0], c_data[index - 1][1]], (0, 255, 0), 2)
    return contour_img


def traj_pred(c_data, img):
    # Extract position and velocity data
    x = c_data[-1][0]
    y = c_data[-1][1]
    vX = c_data[-1][3]
    vY = c_data[-1][4]
    K = c_data[-1][6]

    # Time interval for prediction
    dt = 0.001  # You may need to adjust this based on your specific case

    # Gravitational acceleration
    g = 9.81

    # Predict trajectory using a simple parabolic motion equation
    t = np.arange(0, 2, dt)  # Adjust the time range as needed
    predicted_x = x + vX * t
    predicted_y = y + vY * t + 0.5 * g * 1/K * t ** 2

    # Convert predicted coordinates to integer for drawing with cv2.line
    points = np.array(
        list(zip(predicted_x.astype(int), predicted_y.astype(int))))

    return points


# Variable Set
# Assign upper and lower values from ColorThresholdTesting
ColorLower = (172, 62, 102)
# ColorUpper = (2, 53, 77)
ColorUpper = (179, 255, 255)


# ColorLower = (0, 139, 0)
# ColorUpper = (180, 255, 255)

c_data = []  # Create empty c_data matrice


# ORIGINAL VIDEO
cap = cv.VideoCapture('basketball_cut.mp4')  # Create video element
while True:
    # Get webcam frame
    ret, frame = cap.read()
    if not ret:
        print("No Image Found")
        break
    # Display the Original Image

    cv.imshow('Original Video', frame)
    time.sleep(.05)
    # break loop with q key press
    if cv.waitKey(1) & 0xFF == ord('q'):
        break
# Wait indefinitely until a key is pressed before displaying the masked video
cv.waitKey(0)
cap.release()  # Release the video capture object and reopen it to start from the beginning

# MASKED VIDEO
cap = cv.VideoCapture('basketball_cut.mp4')
while True:
    # Get webcam frame
    ret, frame = cap.read()
    if not ret:
        cv.waitKey(0)
        break

    # convert frame to HSV
    mask = color_masking(frame, ColorLower, ColorUpper)

    # Display the Masked Image in full screen
    cv.imshow('Masked Video', mask)
    time.sleep(.05)

    # break loop with q key press
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cv.waitKey(0)
cap.release()  # Release the video capture object and reopen it to start from the beginning

# CONTOUR VIDEO
cap = cv.VideoCapture('basketball_cut.mp4')
fps = cap.get(cv.CAP_PROP_FPS)
while True:
    # Get video frame
    ret, frame = cap.read()
    if not ret:
        break

    # Display the Contour Image in full screen
    [largest_contour, contour_img] = large_contour_find(
        frame, color_masking(frame, ColorLower, ColorUpper))
    if largest_contour is not None:
        [contour_img, cX, cY] = center_find(largest_contour, contour_img)
        c_data.append([cX, cY, time.time(), None, None, None, None])
        traj_draw(c_data, contour_img)
        if len(c_data) > 1:
            [c_data, contour_img] = velo_draw(c_data, contour_img, fps)
        if len(c_data) == 5:
            cv.waitKey(0)
            points = traj_pred(c_data, contour_img)
            # Draw the trajectory on the image using cv2.line
            cv.polylines(contour_img, [points], isClosed=False, color=(
                255, 0, 0), thickness=2)
            cv.imshow('Contour Video', contour_img)
            cv.waitKey(0)
    else:
        continue

    if len(c_data) > 5:  # print predicted trajectory onto contour_img
        # Draw the trajectory on the image using cv2.line
        cv.polylines(contour_img, [points], isClosed=False, color=(
            255, 0, 0), thickness=2)

    cv.imshow('Contour Video', contour_img)

    if cv.waitKey(1) & 0xFF == ord('q'):  # break loop with q key press
        break

for row in c_data:
    print(row)

cv.waitKey(0)
cap.release()  # Release the video capture object and close all windows
cv.destroyAllWindows()
