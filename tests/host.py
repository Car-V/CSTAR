import asyncio
import websockets
import json

async def handler(websocket):
    """Handles incoming WebSocket connections."""
    print(f"Client connected from {websocket.remote_address}")

    try:
        async for message in websocket:
            data = json.loads(message)  # Parse JSON message
            print(f"Received batch of {len(data)} packets")

            # Log packet data
            for packet in data:
                print(f"Timestamp: {packet['timestamp']}")
                print(f"Position: {packet['positional_data']}")
                print(f"Audio Data (first 10 samples): {packet['audio_data'][:10]}\n")

            # Optional: Send an acknowledgment response
            await websocket.send(json.dumps({"status": "received", "packet_count": len(data)}))

    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")
    except Exception as e:
        print(f"Error: {e}")

async def main():
    """Starts the WebSocket server."""
    server = await websockets.serve(handler, "0.0.0.0", 8765)

    print("WebSocket Server started on ws://0.0.0.0:8765")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())