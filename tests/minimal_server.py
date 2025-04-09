import asyncio
import websockets

async def handle_client(websocket, path):
	print("Client connected")
	try:
		await websocket.recv()
	except Exception as e:
		print(f"Error: {e}")
	finally:
		print("Client disconnected")

async def main():
	host = "127.0.0.1"
	port = 8764
	print(f"Starting server")
	async with websockets.serve(handle_client, host, port):
		await asyncio.Future()

if __name__ == "__main__":
	asyncio.run(main())
