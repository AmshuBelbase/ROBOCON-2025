import cv2 as cv
from cv2 import aruco
import numpy as np

def rvec_to_euler(rvec):
    R, _ = cv.Rodrigues(rvec)
    sy = np.sqrt(R[0,0] ** 2 + R[1,0] ** 2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.degrees([x, y, z])


calib_data_path = r"C:\Users\Happy Home\OneDrive\Desktop\ROBOCON\MultiMatrixlogi.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)
cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]

MARKER_SIZE = 28.1 #cm

marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50)

param_markers = aruco.DetectorParameters()

# cap = cv.VideoCapture(1)
# cap = cv.VideoCapture('http://192.0.0.4:8080/video') #give the server id shown in IP webcam App 

cap = cv.VideoCapture(1)

# Optional: Set resolution (ZED stereo image is usually wide)
# cap.set(cv.CAP_PROP_FRAME_WIDTH, 3840)   # 1280 x 2
# cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ret, frame = cap.read() #1280,720
    if not ret:
        break
    # h, w, _ = frame.shape
    # print(frame.shape)
    # frame=frame[:, :w//2]
    # frame = cv.undistort(frame, cam_mat, dist_coef)
    gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        gray_frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, MARKER_SIZE, cam_mat, dist_coef
        )
        total_markers = range(0, marker_IDs.size)
        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()


            
            # calculate the distance
            distance = np.sqrt(
                tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
            )
            



            # for pose of the marker
            point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
            euler_angles = rvec_to_euler(rVec[0])

            cv.putText(
                frame,
                f"id: {ids[0]} Dist: {round(distance, 2)}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            cv.putText(
                frame,
                f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                bottom_right,
                cv.FONT_HERSHEY_PLAIN,
                1.0,
                (0, 0, 255),
                2,
                cv.LINE_AA,
            )
            cv.putText(frame, str(round(euler_angles[1],2)), (250, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            cv.putText(frame, str(distance), (500, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            cv.putText(frame, str(round(tVec[i][0][2],2)), (20, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
            # print(ids, "  ", corners)
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()