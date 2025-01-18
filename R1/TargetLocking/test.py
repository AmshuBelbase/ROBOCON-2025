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

angle = 0
ser.open()
data = str(angle)
ser.write(data.encode())
# while(True):
#     print(ser.readline().decode('utf-8').strip())
ser.close()