import asyncio
import websockets
import json
from datetime import datetime 

async def connect_to_server():
    uri = "ws://127.0.0.1:8765"


    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to server.")

            for i in range (1000):

                data = i
                # Get timestamp before sending data
                timestamp_sent = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                
                #printing data
                print(f"[{timestamp_sent}] ({i+1}/1000) Sending data: {data}")               
                await websocket.send(json.dumps(data))

                # Get timestamp after receiving data
                response = await websocket.recv()
                timestamp_received = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                received_data = json.loads(response)
                print(f"[{timestamp_received}] ({i+1}/1000) Received processed data: {received_data}")
            

    except websockets.exceptions.ConnectionClosed as e:
        print(f"Connection closed: {e}")
    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    asyncio.run(connect_to_server())




