from ultralytics import YOLO
import math
import cv2
import serial
import time

# #Constants
TEENSY_PORT = "/dev/tty.usbmodem166413901"
BAUD_RATE = 2000000
CONFIDENCE_THRESHOLD = 0.25
FRAME_SKIP = 2  # Skip every N frames to maintain real-time performance
BASKET_WIDTH_CM = 47.2
CONST = 930  # Precomputed constant for distance calculation

# Initialize Serial Connection
ser = serial.Serial(TEENSY_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # Allow the serial connection to initialize
print("Connection established with Teensy.")

# Initialize YOLO Model
model = YOLO('best.pt')

# Initialize Video Capture
cap = cv2.VideoCapture(0)

def calculate_angle(perpendicular, base):
    """Calculate angle in degrees based on perpendicular and base."""
    angle = math.degrees(math.atan(perpendicular / base))
    print(f"Angle: {angle:.2f}°")
    return int(angle)

frame_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # frame_count += 1
    # if frame_count % FRAME_SKIP != 0:
    #     continue  # Skip frames for performance

    # Frame dimensions and thresholds
    frame_width = frame.shape[1]
    frame_height = frame.shape[0]
    right_threshold = frame_width / 2.0 + frame_width * 0.01
    left_threshold = frame_width / 2.0 - frame_width * 0.01

    # YOLO Inference
    results = model.predict(source=frame, conf=CONFIDENCE_THRESHOLD, save=False,device="mps")#56 show=True)
    cv2.imshow("Frame",results[0].plot())

    # Process detections
    for detection in results[0].boxes:
        # Extract detection details
        x_center, y_center, width, height = detection.xywh[0].cpu().numpy()
        confidence = float(detection.conf[0].cpu().numpy())
        class_id = int(detection.cls[0].cpu().numpy())
        class_name = results[0].names.get(class_id, "Unknown")

        if class_name != "basket":
            ser.write(b"n")  # Send default signal
            continue

        # Calculate distance and offset
        unit_length = BASKET_WIDTH_CM / width  # Distance per pixel in cm
        distance = unit_length * CONST
        print(f"Distance to basket: {distance:.2f} cm")

        error_offset = 0
        if x_center > right_threshold:
            error_offset = x_center - frame_width / 2.0
        elif x_center < left_threshold:
            error_offset = -(frame_width / 2.0 - x_center)

        # Send angle to Teensy
        if error_offset != 0:
            angle = -1*calculate_angle(error_offset, frame_height - y_center)
            print(error_offset)
            data = f"{-1*error_offset}\n"
        else:
            data = "o"  # Centered, no angle adjustment needed

            ser.write(data.encode())
    if cv2.waitKey(1)==ord('q'):
        break

# Cleanup
cap.release()
ser.close()
