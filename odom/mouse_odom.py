import pygame

pygame.init()
screen = pygame.display.set_mode((1920, 1080))
pygame.mouse.set_visible(False)  # Hide cursor

virtualY = 0  # Start at any value (e.g., 0, 540, etc.)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEMOTION:
            # Accumulate relative Y movement (event.rel[1] is the delta)
            virtualY += event.rel[1]
            print(f"Unbounded Y: {virtualY}")

pygame.quit()

# from pynput import mouse

# # Store the last known position
# last_x, last_y = None, None
# virtual_x, virtual_y = 0, 0  # Virtual position beyond screen


# def on_move(x, y):
#     global last_x, last_y, virtual_x, virtual_y

#     if last_x is None or last_y is None:
#         last_x, last_y = x, y  # Initialize

#     # Calculate movement delta
#     dx = x - last_x
#     dy = y - last_y

#     # Update the virtual position (ignores screen boundaries)
#     virtual_x += dx
#     virtual_y += dy

#     print(f"Mouse moved to ({virtual_x}, {virtual_y})")

#     # Update last known position
#     last_x, last_y = x, y


# # Start listening to the external mouse
# with mouse.Listener(on_move=on_move) as listener:
#     listener.join()


# from pynput import mouse


# def on_move(x, y):
#     print(f"Mouse moved to ({x}, {y})")


# # Start listening to the external mouse
# with mouse.Listener(on_move=on_move) as listener:
#     listener.join()

# import win32api
# from pynput import mouse

# # Get the number of mice connected


# def get_mouse_count():
#     return win32api.GetSystemMetrics(43)  # 43 is SM_MOUSEPRESENT


# def on_move(x, y):
#     if get_mouse_count() > 1:  # If more than one mouse (external mouse is present)
#         print(f"External Mouse moved to ({x}, {y})")


# with mouse.Listener(on_move=on_move) as listener:
#     listener.join()
