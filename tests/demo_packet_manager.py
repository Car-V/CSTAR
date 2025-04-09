import rclpy
from rclpy.node import Node
import websockets
import asyncio
import json
import time
import random
import threading

class MockPacketManager(Node):
    def __init__(self):
        super().__init__('mock_packet_manager')

        # Set batch interval (500ms)
        self.batch_interval = 0.5
        self.last_batch_time = time.time()

        # WebSocket URL for the host PC (Update with your actual host IP)
        self.websocket_url = "ws://192.168.110.86:8765"
        self.websocket = None

        # Start timer to generate and send mock data
        self.create_timer(self.batch_interval, self.try_publish_packet)

        # Start the asyncio event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.loop_thread.start()

    def run_event_loop(self):
        """Run the asyncio event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def connect_websocket(self):
        """Ensure the WebSocket connection is active"""
        try:
            if self.websocket is None or self.websocket.close_code is not None:
                self.get_logger().info("Connecting to WebSocket server...")
                self.websocket = await websockets.connect(self.websocket_url)
        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {str(e)}")
            self.websocket = None  # Reset to force retry

    def generate_mock_audio_data(self):
        """Generate random audio data (as bytes)"""
        return random.randint(100, 1000) 

    def generate_mock_positional_data(self):
        """Generate random positional data"""
        return {
            "x": round(random.uniform(-5, 5), 2),
            "y": round(random.uniform(-5, 5), 2)
        }

    def try_publish_packet(self):
        """Generate and send a batch of mock packets"""
        current_time = time.time()

        if current_time - self.last_batch_time >= self.batch_interval:
            batch = []
            
            # Generate mock data
            for _ in range(random.randint(1, 5)):  # Send 1 to 5 packets per batch
                packet = {
                    "audio_data": self.generate_mock_audio_data(),
                    "positional_data": self.generate_mock_positional_data(),
                    "timestamp": time.time()
                }
                batch.append(packet)

            if batch:
                asyncio.run_coroutine_threadsafe(self.send_packet(batch), self.loop)

            self.last_batch_time = current_time

    async def send_packet(self, batch):
        """Send mock data to the WebSocket server"""
        await self.connect_websocket()  # Ensure WebSocket is connected

        if self.websocket:
            try:
                await self.websocket.send(json.dumps(batch))
                self.get_logger().info(f"Sent batch of {len(batch)} mock packets")
            except websockets.exceptions.ConnectionClosed as e:
                self.get_logger().error(f"WebSocket closed unexpectedly: {str(e)}")
                self.websocket = None  # Force reconnect next attempt
            except Exception as e:
                self.get_logger().error(f"WebSocket error: {str(e)}")
                self.websocket = None  # Reset connection

    def destroy_node(self):
        """Gracefully close WebSocket and stop the event loop"""
        self.loop.call_soon_threadsafe(self.loop.stop)
        if self.websocket:
            self.loop.run_until_complete(self.websocket.close())
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    packet_manager = MockPacketManager()
    rclpy.spin(packet_manager)
    packet_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()