import asyncio
import websockets

async def test_connection():
	uri = "ws://127.0.0.1:8764"
	try:
		async with websockets.connect(uri):
			print("Connection success")
	except Exception as e:
		print(f"Connection failed: {e}")

asyncio.run(test_connection())
