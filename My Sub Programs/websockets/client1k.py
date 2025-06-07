import asyncio
import websockets
import json


async def connect_to_server():
    uri = "ws://198.168.238.34:9999"

    data = list(range(0,10))  
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to server.")

            await websocket.send(json.dumps(data))
            print("Sent Data:", data)  # Now printed after a successful connection
            
            response = await websocket.recv()
            received_data = json.loads(response)
            print("Received Data:", received_data)

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    asyncio.run(connect_to_server())
