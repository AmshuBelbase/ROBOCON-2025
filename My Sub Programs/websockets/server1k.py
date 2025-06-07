import asyncio
import websockets
import json

async def handle_client(websocket):
    print("Client connected")
    try:
        async for message in websocket:
            data = json.loads(message)
            print(f"Received from client: {data}")  # Print received data

            # No processing, just send back the same data
            print(f"Sending back the same data: {data}")  

            await websocket.send(json.dumps(data))
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

async def main():
    async with websockets.serve(handle_client, "192.168.98.34", 9999):
        print("Server started. Waiting for connections...")
        await asyncio.Future()  # Keeps server running

if __name__ == "__main__":
    asyncio.run(main())
