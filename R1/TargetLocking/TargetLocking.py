from ultralytics import YOLO
import numpy
import math
import cv2
import serial
import time

teensy_port = "/dev/tty.usbmodem166413901"  
baud_rate = 2000000

with serial.Serial(teensy_port, baud_rate, timeout=1) as ser:
    time.sleep(2)  # Give the connection a moment to initialize
    print("Connection established with Teensy.")


model = YOLO('best.pt') 
cap = cv2.VideoCapture(0)

def anglecalc(perpendicular , base):
    x = perpendicular/base*1.0

    angle = math.degrees((math.atan(x)))
    print(f"Angle is: {angle}")
    return int(angle)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame_width = frame.shape[1]
    frame_height = frame.shape[0]
    right_threshold=frame_width/2.0 + frame_width*0.05
    left_threshold = frame_width/2.0 - frame_width*0.05  
    results = model.predict(source = frame,conf=0.25,save=False)#,show=True)
    for i in results:
        for box in i.boxes:
            x_center, y_center, width, height = box.xywh[0].cpu().numpy()
            x_center, y_center, width, height = (
                float(x_center),
                float(y_center),
                float(width),
                float(height),
            )
        
            confidence = float(box.conf[0].cpu().numpy())
            class_id = int(box.cls[0].cpu().numpy())
            class_name = results[0].names.get(class_id, "Unknown")
            
            if class_name != "basket":
                ser.open()
                data = "\n"
                ser.write(data.encode())
                # while(True):
                #     print(ser.readline().decode('utf-8').strip())
                ser.close()

            else:
                error_offset = 0

                unit_length = 47.2/ width #returns in cm (47.7 cm is outer diameter of hoop)
                const = 930
                distance = unit_length * const
                print(f"Distance to basket: {distance:.2f} centimeters")

                if x_center>right_threshold:
                    error_offset = x_center - frame_width/2.0
                    print(f"Offsetted error: {error_offset} px")
                elif x_center<left_threshold:
                    error_offset = -(frame_width/2.0 - x_center)
                    print(f"Offsetted error: {error_offset} px")
                if error_offset !=0:
                    angle = anglecalc(error_offset,frame_height-y_center)
                    ser.open()
                    data = f"{angle}\n"
                    ser.write(data.encode())
                    # while(True):
                    #     print(ser.readline().decode('utf-8').strip())
                    ser.close()
                else :
                        ser.open()
                        data = "\n"
                        ser.write(data.encode())
                        # while(True):
                        #     print(ser.readline().decode('utf-8').strip())
                        ser.close()
                    
                #to calculate constant
                f=500/unit_length #for cm #from 5m away
                print(f)
            
                
                