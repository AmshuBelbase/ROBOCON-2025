import numpy as np
import matplotlib.pyplot as plt
import pygame
import serial

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
    running = True
    while running:
        screen.fill(WHITE)

        # visualize_path(source, destination, steps)
        coords = read_serial_data()

        if coords:
            x, y, auto_flag = coords

            if auto_flag == 1:
                source = (x, y)
                xf = x  # to comment
                yf = y  # to comment
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


run_visualization()


'''X=xcosa+ysina
Y=ycosa-xsina
a is the angle (theta)'''
