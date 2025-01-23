import serial
import time

# Adjust your serial port and baud rate accordingly
ser = serial.Serial('COM8', 9600)  
time.sleep(2)  # Wait for the serial connection to establish

def send_data(data):
    """Function to send data to the Pico."""
    ser.write((data + '\n').encode('utf-8'))
    ser.flush() 
    print(f"Sent: {data}")

try:
    while True:
        # Step 1: Send data to the Pico
        # You can modify this to get user input or automate sending
        data_to_send = input("Enter data to send (use # to separate): ")
        send_data(data_to_send)
        
        
        time.sleep(1)  # Add a slight delay to allow for response

        # Step 2: Check if there is incoming data from the Pico
        if ser.in_waiting > 0:
            # Read the incoming data from the Pico
            received_data = ser.readline().decode('utf-8').strip()
            
            # Split the data using '#' and store it in a list
            values = received_data.split('#')
            
            # Print each value with its index
            print("Received data:")
            for i, value in enumerate(values):
                print(f"Value {i + 1}: {value}")

except KeyboardInterrupt:
    print("Terminating...")

finally:
    ser.close()

