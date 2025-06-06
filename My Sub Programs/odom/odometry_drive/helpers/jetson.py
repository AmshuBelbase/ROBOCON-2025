from flask import Flask, request, jsonify
import threading
from time import sleep
import requests
import json

app = Flask(__name__)

xf, yf = 0, 0
URL = "http://192.168.222.196:5000/receive"


def send_data(x, y, a):
    global URL

    data = {
        "x": x,
        "y": y,
        "angle": a
    }
    # Send POST request
    response = requests.post(URL, json=data)
    # Print response
    print("Response:", response.json())


@app.route('/receive', methods=['POST'])
def receive_data():
    global xf, yf
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON data received"}), 400

        # Process the received data
        print("Received Data:", data)
        xf = data['x']
        yf = data['y']

        # update xf, yf
        return jsonify({"message": "Data received successfully", "data": data}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    flask_thread = threading.Thread(target=app.run, kwargs={
        "host": "0.0.0.0", "port": 5000, "debug": False, "use_reloader": False}, daemon=True)
    flask_thread.start()
    # app.run(host="0.0.0.0", port=5000, debug=True)

    send_data()
