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

SERIAL_PORT = "COM4"
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

trail = []


def read_serial_data():
    try:
        data = ser.readline().decode('utf-8').strip()
        if data:
            x, y = map(float, data.split(","))  # "x,y"
            return x/10, y/10
    except:
        return None


def run_visualization():
    running = True
    while running:
        screen.fill(WHITE)

        coords = read_serial_data()
        if coords:
            x, y = coords
            trail.append((WIDTH // 2 + x, HEIGHT // 2 + y))

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
