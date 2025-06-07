import asyncio
import websockets
import json
from datetime import datetime  # Import datetime for timestamps

async def handle_client(websocket):
    print("Client connected")
    try:
        async for message in websocket:
            timestamp_received = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")  # Timestamp of received data
            data = json.loads(message)
            print(f"[{timestamp_received}] Received from client: {data}")  

            # No processing, just send back the same data
            timestamp_sent = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")  # Timestamp of sent data
            print(f"[{timestamp_sent}] Sending back the same data: {data}")  

            await websocket.send(json.dumps(data))
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

async def main():
    async with websockets.serve(handle_client, "127.0.0.1", 8765):
        print("Server started. Waiting for connections...")
        await asyncio.Future()  # Keeps server running

if __name__ == "__main__":
    asyncio.run(main())
