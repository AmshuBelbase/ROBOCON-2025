import cv2
import numpy as np
from pupil_apriltags import Detector

# === Load camera calibration ===
calib_data = np.load('MultiMatrix.npz')  # Replace with your filename
camera_matrix = calib_data['camMatrix']
dist_coeffs = calib_data['distCoef']

# === Extract intrinsics for AprilTag detector ===
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
cx = camera_matrix[0, 2]
cy = camera_matrix[1, 2]
camera_params = [fx, fy, cx, cy]

# === Tag parameters ===
tag_size = 5  # Real-world tag size in centimeters (e.g., 15 cm)

# === Initialize AprilTag detector ===
detector = Detector(families='tag36h11',
                    nthreads=1,
                    quad_decimate=1.0,
                    quad_sigma=0.0,
                    refine_edges=1,
                    decode_sharpening=0.25,
                    debug=0)

# === Start video capture ===
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to capture frame.")
        break

    # === Optional: Undistort the frame for better accuracy ===
    frame_undistorted = frame#cv2.undistort(frame, camera_matrix, dist_coeffs)
    gray = cv2.cvtColor(frame_undistorted, cv2.COLOR_BGR2GRAY)

    # === Detect AprilTags ===
    tags = detector.detect(gray, estimate_tag_pose=True,
                           camera_params=camera_params, tag_size=tag_size)

    for tag in tags:
        tag_id = tag.tag_id
        pose_t = tag.pose_t  # [x, y, z] translation in meters
        pose_r = tag.pose_R  # 3x3 rotation matrix

        distance = np.linalg.norm(pose_t)

        # Draw tag center and info
        center = tuple(np.int32(tag.center))
        cv2.circle(frame_undistorted, center, 6, (0, 255, 0), -1)
        cv2.putText(frame_undistorted, f"ID: {tag_id}  Dist: {distance:.2f}m",
                    (center[0] + 10, center[1]), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 0, 0), 2)

        # Draw tag border
        for i in range(4):
            pt1 = tuple(np.int32(tag.corners[i]))
            pt2 = tuple(np.int32(tag.corners[(i + 1) % 4]))
            cv2.line(frame_undistorted, pt1, pt2, (0, 255, 255), 2)

    # Show the result
    cv2.imshow("AprilTag Detection", frame_undistorted)
    if cv2.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
 