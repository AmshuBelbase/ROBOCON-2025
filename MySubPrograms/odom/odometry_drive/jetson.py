import numpy as np
import serial
from flask import Flask, request, jsonify
import threading
import requests

SERIAL_PORT, BAUD_RATE = "COM3", 115200
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

app = Flask(__name__)

xf, yf = 0, 0
xn, yn = 0, 0
xc, yc = 0, 0

URL = "http://192.168.222.133:5000/receive"


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

def divide_path_step_cm(source, destination, steps):
    global xn, yn
    source = np.array(source)
    destination = np.array(destination)

    # Compute total distance
    direction = destination - source
    total_distance = np.linalg.norm(direction)

    if total_distance == 0:  # If source and destination are the same
        xn = 0
        yn = 0
        angle = 0

        data = f"{xn},{yn},{angle}\n"
        ser.write(data.encode())
        print(f"Sent: {data.strip()}")

        return

    # Normalize direction vector
    unit_direction = direction / total_distance

    # Generate points at step intervals
    points = [tuple(source)]
    for d in np.arange(steps, total_distance, steps):
        new_point = source + unit_direction * d
        points.append(tuple(new_point))

    # Append destination if remaining distance is less than 2 cm
    if total_distance % 2 != 0:
        points.append(tuple(destination))

    lxy = points[1]

    xn = lxy[0]
    yn = lxy[1]

    xn -= source[0]
    yn -= source[1]

    xn = int(xn*4)
    yn = int(yn*4)
    angle = 0

    data = f"{xn},{yn},{angle}\n"
    ser.write(data.encode())
    print(f"Sent: {data.strip()}")

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
        print(coords)
        if coords:
            xc, yc, angle, auto_flag = coords
            if auto_flag == 1:  # if autonomous mode, send next waypoint to teensy
                source = (xc, yc)
                destination = (xf, yf)
                steps = 10
                divide_path_step_cm(source, destination, steps)

            if c % 50 == 0:
                send_to_laptop(angle)  # send current position to laptop
                c = 0
            c += 1


if __name__ == '__main__':
    flask_thread = threading.Thread(target=app.run, kwargs={
                                    "host": "0.0.0.0", "port": 5000, "debug": False}, daemon=True)
    flask_thread.start()

    # Run the visualization loop in the main thread
    receive_send()
    ser.close()
