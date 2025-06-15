import pydirectinput

# Setup a function to read raw mouse data


def get_raw_mouse_position():
    # Initialize DirectInput
    mouse = pydirectinput.position()

    # Get raw movement data (dx, dy, dz)
    # dx, dy, dz = mouse.get_position()

    # Print the relative movement (dx, dy)
    # print(f"Mouse moved by: dx = {dx}, dy = {dy}, dz = {dz}")


# Call the function continuously to monitor the movement
try:
    while True:
        get_raw_mouse_position()
except KeyboardInterrupt:
    print("Exiting...")
