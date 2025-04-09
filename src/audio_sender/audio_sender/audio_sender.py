# FOR SENDING AUDIO FILES FROM MIC TO CATHERINE, HERS IS AUDIORECEIVER.PY

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

#"ws://172.20.10.3:8764"#"ws://192.168.7.86:8764" # will need to configure

class AudioSender(Node):
    def __init__(self):
        super().__init__('audio_sender')
        self.get_logger().info('Audio sender node initialized')

        self.position_before = None
        #self.tf_buffer = tf2_ros.Buffer()
        # #self.tf_buffer.set_max_size(500)
        #self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # self.latest_transform = None
        #self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.create_subscription(PoseStamped, '/slam_toolbox/pose', self.pose_callback, 10)
        #self.timer = self.create_timer(5.0, self.get_pose) #HERE 
        self.websocket_url = "ws://192.168.7.86:8764" #"ws://127.0.0.1:8764" #"ws://192.168.7.29:8764" #
        self.websocket = None
        # self.pose_sub = Subscriber(self, PoseStamped, '/slam_toolbox/pose')
        # self.sync = TimeSynchronizer([self.pose_sub], 10)
        # self.sync.registerCallback(self.pose_callback)
        self.timer = self.create_timer(2.0, self.record_and_send_audio) # triggers every 5 sec

        # Start the asyncio event loop in a separate thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.run_event_loop, daemon=True)
        self.loop_thread.start()
    # is this 5 seconds after completes or after called?
	# what if we made the timer 5.5?
    #def get_pose(self):
    #     try:
    #         transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    #         pose_msg = PoseStamped()
    #         pose_msg.pose.position.x = transform.transform.translation.x
    #         pose_msg.pose.position.y = transform.transform.translation.y
             #pose_msg.header.stamp = self.get_clock().now().to_msg()
    #         self.current_position = (pose_msg.pose.position.x, pose_msg.pose.position.y)
    #         self.get_logger().info(f"Received odometry data: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}")
    #     except:
    #         pass
    #HERE
    # def odom_callback(self, msg):
    #    position = msg.pose.pose.position
    #    self.current_position = (position.x, position.y)
    #    self.get_logger().info(f"Received odometry data: {position.x}, {position.y}")
    def pose_callback(self, msg):
        self.current_position = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Received odometry data: {self.current_position[0]}, {self.current_position[1]}")

    # def pose_callback(self, msg):
    #     try:
    #         if self.tf_buffer.can_transform('map', 'base_link', msg.header.stamp):
    #             transform = self.tf_buffer.lookup_transform('map', 'base_link', msg.header.stamp)
    #             self.position_before = (transform.transform.translation.x, transform.transform.translation.y)
    #             self.get_logger().info(f"Received odometry data: {self.position_before[0]}, {self.position_before[1]}")
    #     except:
    #         self.get_logger().info("Failed")

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
#        try:
        await self.connect_websocket()
        if self.websocket:
            try:
                with open(file_name, 'rb') as audio_file:
                    data = audio_file.read()
                    payload = {'position_before': self.position_before, 'audio': data.hex()} #, 'position_after': self.position_after}
                    await self.websocket.send(json.dumps(payload))
                    self.get_logger().info(f'{file_name} sent to {self.websocket_url}')
            except Exception as e:
                self.get_logger().error(f'Error sending audio: {str(e)}')
                self.websocket = None


    def record_and_send_audio(self):# was async
        #for i in range(3): # we will eventually keep overriding, so should go back to 0
        self.position_before = getattr(self, 'current_position', None)

        # if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
        #     transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        # else:
        #     time.sleep(0.05)
        #     transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        # pose_msg = PoseStamped()
        # pose_msg.pose.position.x = transform.transform.translation.x
        # pose_msg.pose.position.y = transform.transform.translation.y
        #pose_msg.header.stamp = self.get_clock().now().to_msg()
        #self.position_before = (pose_msg.pose.position.x, pose_msg.pose.position.y)
        #self.get_logger().info(f"Received odometry data: {pose_msg.pose.position.x}, {pose_msg.pose.position.y}") 

        self.get_logger().info(f"Position before recording: {self.position_before}")

        file_name = f"audio.wav" #f"audio_{i+1}.wav"
        self.get_logger().info(f"Recording: {file_name}")
        os.system(f"arecord -r 48000 -f S16_LE -d 2 {file_name}")
        self.get_logger().info(f"Recording {file_name} completed")

        #rclpy.spin_once(self, timeout_sec=0.1) #time.sleep(0.1) not updating for some reason

        #self.position_after = getattr(self, 'current_position', None)
        #self.get_logger().info(f"Position after recording: {self.position_after}") 

        #loop = asyncio.get_event_loop()
        #await self.send_audio(file_name)
        self.loop.call_soon_threadsafe(asyncio.create_task, self.send_audio(file_name))

    def destroy_node(self):
        """Gracefully close WebSocket and stop the event loop"""
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()
        if self.websocket:
            self.loop.run_until_complete(self.websocket.close())
        super().destroy_node()

    # def record_audio(file_name):
    #     command = f"arecord -r 48000 -f S16_LE -d 5 {file_name}"
    #     os.system(command)


    # def run_event_loop():
    #     """Run the asyncio event loop in a separate thread"""
    #     asyncio.set_event_loop(loop)
    #     loop.run_forever()


    # async def send_packet(self):
    #     """Send mock data to the WebSocket server"""
    #     await self.connect_websocket()  # Ensure WebSocket is connected

    #     if websocket:
    #         try:
    #             await self.send_audio()
    #             self.get_logger().info(f"Sent audio file")
    #         except websockets.exceptions.ConnectionClosed as e:
    #             self.get_logger().error(f"WebSocket closed unexpectedly: {str(e)}")
    #             websocket = None  # Force reconnect next attempt
    #         except Exception as e:
    #             self.get_logger().error(f"WebSocket error: {str(e)}")
    #             websocket = None  # Reset connection

    # async def driver_func():
    #     for i in range(3):  # You can increase the range if you want more recordings
    #         file_name = f"audio_{i+1}.wav"
    #         print(f"Recording: {file_name}")
    #         record_audio(file_name)
    #         print(f"Recording {file_name} completed.")
    #         await send_audio(file_name, websocket_url)
    #         #await asyncio.sleep(0.001)  # Adding a 1-millisecond delay

def main(args=None):
    rclpy.init()
    node = AudioSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
