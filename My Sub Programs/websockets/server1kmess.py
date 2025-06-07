import asyncio
import websockets
import json
from datetime import datetime  

async def handle_client(websocket):
    print("Client connected")
    try:
        for i in range (1000):
            async for message in websocket:
                 #message = await websocket.recv()
                timestamp_received = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                data = json.loads(message)
                print(f"[{timestamp_received}] ({data}/1000) Received from client: {data}")
                
                #processing data

                processed_data = data +1000
                timestamp_sent = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
                print(f"[{timestamp_sent}] ({data}/1000) Sending processed data: {processed_data}")
                await websocket.send(json.dumps(processed_data))

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")


async def main():
    async with websockets.serve(handle_client, "127.0.0.1", 8765):
        print("Server started. Waiting for connections...")
        await asyncio.Future()  # Keeps server running

if __name__ == "__main__":
    asyncio.run(main())



