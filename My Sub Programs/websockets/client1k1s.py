import asyncio
import websockets
import json
from datetime import datetime  # Import datetime for timestamps

async def connect_to_server():
    uri = "ws://192.168.238.34:9999"

    # Generate a list from -500 to 498
    data = list(range(-500, 499))  
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to server.")

            # Get timestamp before sending data
            timestamp_sent = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            print(f"[{timestamp_sent}] Sending Data: {data}")  

            await websocket.send(json.dumps(data))

            # Get timestamp after receiving data
            response = await websocket.recv()
            timestamp_received = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
            received_data = json.loads(response)

            print(f"[{timestamp_received}] Received Data: {received_data}")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(connect_to_server())
