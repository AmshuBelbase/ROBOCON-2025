import math
import cv2 as cv
import numpy as np
import time


def color_masking(frame, color_lower, color_upper):
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, color_lower, color_upper)
    mask = cv.erode(mask, None, iterations=5)
    mask = cv.dilate(mask, None, iterations=5)
    return mask


def large_contour_find(frame, mask):
    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if cnts:
        largest_contour = max(cnts, key=cv.contourArea)
        cv.drawContours(frame, [largest_contour], -1, (0, 255, 0), 3)
        return largest_contour
    return None


def center_find(cnt, img):
    M = cv.moments(cnt)
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        cv.circle(img, (cX, cY), 7, (255, 255, 255), -1)
        cv.putText(img, "center", (cX - 20, cY - 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return cX, cY
    return None, None


def velo_draw(c_data, fps):
    dt = 1 / fps
    vX = (c_data[-1][0] - c_data[-2][0]) / dt
    vY = (c_data[-1][1] - c_data[-2][1]) / dt
    aY = None
    if len(c_data) > 3:
        K = -(-9.81 * dt) / (vY - c_data[-2][4])
        aY = K * (vY - c_data[-2][4]) / dt
    c_data[-1][3] = vX
    c_data[-1][4] = vY
    c_data[-1][5] = aY
    return c_data


def traj_draw(c_data, img):
    for index, data in enumerate(c_data):
        cv.circle(img, (data[0], data[1]), 5, (0, 255, 0), -1)
        if index > 0:
            cv.line(img, (data[0], data[1]), (c_data[index - 1]
                    [0], c_data[index - 1][1]), (0, 255, 0), 2)


def traj_pred(c_data, img):
    x, y = c_data[-1][0], c_data[-1][1]
    vX, vY = c_data[-1][3], c_data[-1][4]
    K = c_data[-1][6] if c_data[-1][6] is not None else 1
    dt = 0.01
    g = 9.81
    t = np.arange(0, 2, dt)
    predicted_x = x + vX * t
    predicted_y = y + vY * t + 0.5 * g * (1/K) * t ** 2
    points = np.array(
        list(zip(predicted_x.astype(int), predicted_y.astype(int))))
    cv.polylines(img, [points], isClosed=False, color=(255, 0, 0), thickness=2)


# Color range for masking
ColorLower = (0, 139, 0)
ColorUpper = (180, 255, 255)
c_data = []

cap = cv.VideoCapture('Tomato.mp4')
fps = cap.get(cv.CAP_PROP_FPS)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    mask = color_masking(frame, ColorLower, ColorUpper)
    largest_contour = large_contour_find(frame, mask)
    if largest_contour is not None:
        cX, cY = center_find(largest_contour, frame)
        if cX is not None and cY is not None:
            c_data.append([cX, cY, time.time(), None, None, None, None])
            if len(c_data) > 1:
                c_data = velo_draw(c_data, fps)
                traj_draw(c_data, frame)
            if len(c_data) >= 5:
                traj_pred(c_data, frame)

    cv.imshow('Unified Video Output', frame)
    time.sleep(0.3)
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv.destroyAllWindows()
