# audio_sender node for collecting mic samples and positions from pose_listener
# For configuration, update self.websocket_url to the IP holding the host_program.py, on the same network

import os
import time
import asyncio
import websockets
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from message_filters import Subscriber, TimeSynchronizer
import threading
import json
import tf2_ros

class AudioSender(Node):
    def __init__(self):
        super().__init__('audio_sender')
        self.get_logger().info('Audio sender node initialized')

        self.position_before = None
        self.create_subscription(PoseStamped, '/slam_toolbox/pose', self.pose_callback, 10)
  	#self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.websocket_url = "ws://127.0.0.1:8764" #localhost
        self.websocket = None
        self.timer = self.create_timer(2.0, self.record_and_send_audio) # triggers every 5 sec

        # Start the asyncio event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.loop_thread.start()

    # def odom_callback(self, msg):
    #    position = msg.pose.pose.position
    #    self.current_position = (position.x, position.y)
    #    self.get_logger().info(f"Received odometry data: {position.x}, {position.y}")

    def pose_callback(self, msg):
        self.current_position = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Received odometry data: {self.current_position[0]}, {self.current_position[1]}")

    def run_event_loop(self):
        """Run the asyncio event loop in a separate thread"""
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    async def connect_websocket(self):
        """Ensure the WebSocket connection is active"""
        try:
            if self.websocket is None: # or self.websocket.close_code is not None:
                self.get_logger().info("Connecting to WebSocket server...")
                self.websocket = await websockets.connect(self.websocket_url)
        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {str(e)}")
            self.websocket = None  # Reset to force retry

    async def send_audio(self, file_name):
        await self.connect_websocket()
        if self.websocket:
            try:
                with open(file_name, 'rb') as audio_file:
                    data = audio_file.read()
                    payload = {'position_before': self.position_before, 'audio': data.hex()}
                    await self.websocket.send(json.dumps(payload))
                    self.get_logger().info(f'{file_name} sent to {self.websocket_url}')
            except Exception as e:
                self.get_logger().error(f'Error sending audio: {str(e)}')
                self.websocket = None

    def record_and_send_audio(self):
        self.position_before = getattr(self, 'current_position', None)
        self.get_logger().info(f"Position before recording: {self.position_before}")
        file_name = f"audio.wav" #f"audio_{i+1}.wav"
        self.get_logger().info(f"Recording: {file_name}")
        os.system(f"arecord -r 48000 -f S16_LE -d 2 {file_name}")
        self.get_logger().info(f"Recording {file_name} completed")
        self.loop.call_soon_threadsafe(asyncio.create_task, self.send_audio(file_name))

    def destroy_node(self):
        """Gracefully close WebSocket and stop the event loop"""
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()
        if self.websocket:
            self.loop.run_until_complete(self.websocket.close())
        super().destroy_node()


def main(args=None):
    rclpy.init()
    node = AudioSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
