import numpy as np
import matplotlib.pyplot as plt
import pygame
import serial
from flask import Flask, request, jsonify
import threading

app = Flask(__name__)


pygame.init()

WIDTH, HEIGHT = 2000, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mouse Movement from Serial")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

SERIAL_PORT = "COM3"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

trail = []

xf = 0
yf = 0


def divide_path_step_cm(source, destination, steps):
    source = np.array(source)
    destination = np.array(destination)

    # Compute total distance
    direction = destination - source
    total_distance = np.linalg.norm(direction)

    if total_distance == 0:  # If source and destination are the same
        x = 0.0
        y = 0.0
        angle = 0.0

        data = f"{x},{y},{angle}\n"
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

    x = lxy[0]
    y = lxy[1]

    x -= source[0]
    y -= source[1]

    x = x*4
    y = y*4
    angle = 0

    data = f"{x},{y},{angle}\n"
    ser.write(data.encode())
    print(f"Sent: {data.strip()}")

    # return points


def read_serial_data():
    try:
        data = ser.readline().decode('utf-8').strip()
        if data:
            auto_flag, x, y, a = map(float, data.split(","))
            return x/3, y/3, auto_flag
    except:
        return None


def run_visualization():
    global xf, yf
    running = True
    while running:
        screen.fill(WHITE)

        # visualize_path(source, destination, steps)
        coords = read_serial_data()

        if coords:
            x, y, auto_flag = coords

            if auto_flag == 1:
                source = (x, y)

                destination = (xf, yf)
                steps = 10
                divide_path_step_cm(source, destination, steps)

            trail.append((WIDTH // 2+x, HEIGHT // 2-y))
            print(x*3, ',', y*3)

        if len(trail) > 1:
            pygame.draw.lines(screen, RED, False, trail, 3)

        if trail:
            font = pygame.font.Font(None, 36)
            text = font.render(f"Position: {trail[-1]}", True, BLACK)
            screen.blit(text, (10, 10))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

    pygame.quit()
    ser.close()


@app.route('/receive', methods=['POST'])
def receive_data():
    global xf, yf
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON data received"}), 400

        # Process the received data
        print("Received Data:", data)

        # update xf, yf
        return jsonify({"message": "Data received successfully", "data": data}), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500


if __name__ == '__main__':
    flask_thread = threading.Thread(target=app.run, kwargs={
                                    "host": "0.0.0.0", "port": 5000, "debug": True}, daemon=True)
    flask_thread.start()

    # Run the visualization loop in the main thread
    run_visualization()
