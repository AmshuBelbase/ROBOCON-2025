import numpy as np
import serial
from flask import Flask, request, jsonify
import threading
import requests
import math
SERIAL_PORT, BAUD_RATE = "COM14", 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

app = Flask(__name__)

xf, yf = 0, 0
xn, yn = 0, 0
xc, yc = 0, 0

URL = "http://192.168.118.29:5000/receive"  # print(coords)


# update final destination after receiving from laptop

@app.route('/from_laptop_to_jetson', methods=['POST'])
def from_laptop_to_jetson():
    global xf, yf
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON data received"}), 400

        print("Received Data:", data)
        xf = data['x']
        yf = data['y']

        return jsonify({"message": "Data received successfully", "data": data}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# send bot position to laptop

def send_to_laptop(a):
    global URL, xc, yc

    data = {
        "x": xc,
        "y": yc,
        "angle": a
    }
    response = requests.post(URL, json=data)
    print("Response:", response.json())


# when autonomous mode, calculate next way point and send to teensy using serial; 0,0 if at destination

def get_send_path(source, destination, angle):
    source = np.array(source)
    destination = np.array(destination)

    # Compute total distance
    direction = destination - source
    total_distance = np.linalg.norm(direction)
    print(
        f"Source: {source}, Destination: {destination}, Distance: {total_distance: .2f} cm, Angle: {angle}")

    if total_distance < 5:  # If source and destination are the same
        xn = 0
        yn = 0
        angle = 0

        data = f"{xn},{yn},{angle}\n"
        ser.write(data.encode())
        print(f"Sent: {data.strip()}")

        return

    y = destination[1] - source[1]
    x = destination[0] - source[0]

    x1 = (math.cos(angle/57.2957795)*x)-(math.sin(angle/57.2957795)*y)
    y1 = (math.sin(angle/57.2957795)*x)+(math.cos(angle/57.2957795)*y)

    scale = 0.002
    y_scaled = y1 * total_distance * scale
    x_scaled = x1 * total_distance * scale

    limits = [6, 25, 100]

    x_scaled = round(x_scaled, 2)
    y_scaled = round(y_scaled, 2)
    print(f"Before Small: {x_scaled},{y_scaled}")

    if abs(x_scaled) <= abs(y_scaled):
        if x_scaled != 0:
            term = x_scaled
        else:
            term = y_scaled
    elif abs(x_scaled) > abs(y_scaled):
        if y_scaled != 0:
            term = y_scaled
        else:
            term = x_scaled

    if total_distance < limits[2]:
        lim = limits[2]*(total_distance/200)
        limits[1] = lim if lim > limits[0] else limits[0]
    if abs(term) < limits[0]:
        x_scaled = math.ceil(round(x_scaled * (limits[0]/abs(term)), 2))
        y_scaled = math.ceil(round(y_scaled * (limits[0]/abs(term)), 2))

    print(f"After Small: {x_scaled},{y_scaled}")

    term = x_scaled if abs(x_scaled) > abs(y_scaled) else y_scaled

    if abs(term) > limits[1]:
        x_scaled = math.ceil(round(x_scaled * (limits[1]/abs(term)), 2))
        y_scaled = math.ceil(round(y_scaled * (limits[1]/abs(term)), 2))

    print(f"After Large: {x_scaled},{y_scaled}")

    x_scaled = x_scaled if abs(x_scaled) >= limits[0] else 0
    y_scaled = y_scaled if abs(y_scaled) >= limits[0] else 0

    scaled_target = (round(x_scaled, 0), round(y_scaled, 0))
    print(f"Controller Replica: {scaled_target}")

    xn = scaled_target[0]
    yn = -scaled_target[1]
    angle = 0

    data = f"{xn},{yn},{angle}\n"
    ser.write(data.encode())
    print(f"Sent: {data.strip()}\n")

    return


# read odom data coming from teensy using serial

def read_serial_data():
    try:
        data = ser.readline().decode('utf-8').strip()
        if data:
            auto_flag, x, y, angle = map(float, data.split(","))
            return x, y, angle, auto_flag
    except:
        return None


# receive from teensy
# if autonomous mode, send next waypoint to teensy
# send current position to laptop

def receive_send():
    global xf, yf, xc, yc
    c = 0
    while True:
        coords = read_serial_data()  # receive from teensy
        if coords:
            print(coords)
            # xc, yc, angle, auto_flag = coords
            auto_flag, xc, yc, angle, yv, xx, xy = coords

            if auto_flag == 1:  # if autonomous mode, send next waypoint to teensy
                source = (xc, yc)
                destination = (xf, yf)
                # steps = 10
                print("Sending Procedure")
                get_send_path(source, destination, 360-angle)

            # if c % 50 == 0:
            #     send_to_laptop(angle)  # send current position to laptop
            #     c = 0
            # c += 1


if __name__ == '__main__':
    flask_thread = threading.Thread(target=app.run, kwargs={
                                    "host": "0.0.0.0", "port": 5000, "debug": False}, daemon=True)
    flask_thread.start()

    # Run the visualization loop in the main thread
    receive_send()
    ser.close()
