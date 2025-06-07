import cv2

def list_connected_cameras():
    index = 0
    camera_indices = []

    while True:
        # Try to open the camera at the current index
        cap = cv2.VideoCapture(index)
        if cap.read()[0]:  # Check if the camera opens successfully
            camera_indices.append(index)
            cap.release()
        else:
            cap.release()
            break  # No more cameras to check
        index += 1

    return camera_indices

if __name__ == "__main__":
    cameras = list_connected_cameras()
    if cameras:
        print("Connected camera indices:", cameras)
    else:
        print("No cameras connected.")