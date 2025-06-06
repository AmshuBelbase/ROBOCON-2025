from flask import Flask, request, jsonify
import threading
import requests

import pygame

pygame.init()

WIDTH, HEIGHT = 2000, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Mouse Movement from Serial")

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

trail = []

app = Flask(__name__)

xc, yc = 0, 0


# update final destination

def send_fun():
    URL = "http://192.168.118.238:5000/from_laptop_to_jetson"

    while True:
        yf = int(input("Enter Y:"))
        xf = int(input("Enter X:"))
        # xf = 0

        data = {
            "x": xf,
            "y": yf,
            "angle": 45
        }
        response = requests.post(URL, json=data)
        print("Response:", response.json())


# receive and update bot's current position

@app.route('/receive', methods=['POST'])
def receive_data():
    global xc, yc
    try:
        data = request.get_json()
        if not data:
            return jsonify({"error": "No JSON data received"}), 400

        xc = data['x']
        yc = data['y']

        return jsonify({"message": "Received"}), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500


def run_visualization():
    global xc, yc
    running = True
    while running:
        screen.fill(WHITE)

        if xc:
            x, y = xc/3, yc/3
            trail.append((WIDTH // 2+x, HEIGHT // 2-y))

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


if __name__ == '__main__':
    flask_thread = threading.Thread(target=app.run, kwargs={
        "host": "0.0.0.0", "port": 5000, "debug": False, "use_reloader": False}, daemon=True)
    flask_thread.start()

    send_thread = threading.Thread(target=send_fun, daemon=True)
    send_thread.start()

    run_visualization()
