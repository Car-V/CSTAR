import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from rclpy import time

from turtlesim.srv import Spawn


class PoseListener(Node):

    def __init__(self):
        super().__init__('pose_listener')

        # Declare and acquire target_frame parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'base_link').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create a client to spawn a turtle
        #self.spawner = self.create_client(Spawn, 'spawn')
        # Boolean values to store the information
        # if the service for spawning turtle is available
        #self.turtle_spawning_service_ready = False
        # if the turtle was successfully spawned
        #self.turtle_spawned = False

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(PoseStamped, 'slam_toolbox/pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'map' #target2

                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        # scale_rotation_rate = 1.0
        # msg.angular.z = scale_rotation_rate * math.atan2(
        #     t.transform.translation.y,
        #     t.transform.translation.x)

        # scale_forward_speed = 0.5
        # msg.linear.x = scale_forward_speed * math.sqrt(
        #     t.transform.translation.x ** 2 +
        #     t.transform.translation.y ** 2)
        # Extract translation from transform
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z

        # Extract rotation from transform
        msg.pose.orientation.x = t.transform.rotation.x
        msg.pose.orientation.y = t.transform.rotation.y
        msg.pose.orientation.z = t.transform.rotation.z
        msg.pose.orientation.w = t.transform.rotation.w

        self.publisher.publish(msg)
        self.get_logger().info(f'Published pose: {msg.pose}')


def main():
    rclpy.init()
    node = PoseListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from tf2_ros import TransformException
# import tf2_ros
# import tf2_geometry_msgs
# import math

# class PoseListener(Node):
#     def __init__(self):
#         super().__init__('pose_listener')
#         self.tf_buffer = tf2_ros.Buffer(self)
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
#         self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 10)

#     def publish_pose(self):
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 'map', 'base_link', rclpy.time.Time(), rclpy.Duration(seconds=0.1)
#             )
#         except TransformException as ex:
#             self.get_logger().warning(f'No transform available: {ex}')
#             return

#         pose_stamped = PoseStamped()
#         pose_stamped.header.stamp = self.get_clock().now().to_msg()
#         pose_stamped.header.frame_id = 'map'

#         # Extract translation from transform
#         pose_stamped.pose.position.x = transform.transform.translation.x
#         pose_stamped.pose.position.y = transform.transform.translation.y
#         pose_stamped.pose.position.z = transform.transform.translation.z

#         # Extract rotation from transform
#         pose_stamped.pose.orientation.x = transform.transform.rotation.x
#         pose_stamped.pose.orientation.y = transform.transform.rotation.y
#         pose_stamped.pose.orientation.z = transform.transform.rotation.z
#         pose_stamped.pose.orientation.w = transform.transform.rotation.w

#         self.pose_publisher.publish(pose_stamped)
#         self.get_logger().info(f'Published pose: {pose_stamped.pose}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = PoseListener()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# import tf2_ros
# from geometry_msgs.msg import PoseStamped

# class PoseListenerNode(Node):
#     def __init__(self):
#         super().__init__('pose_listener')
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
#         self.pose_pub = self.create_publisher(PoseStamped, '/slam_toolbox/pose', 10)
#         self.timer = self.create_timer(0.1, self.publish_pose)

#     # def pose_callback(self, msg):
#     #     try:
#     #         if self.tf_buffer.can_transform('map', 'base_link', msg.header.stamp):
#     #             transform = self.tf_buffer.lookup_transform('map', 'base_link', msg.header.stamp)

#     #             x = transform.transform.translation.x
#     #             y = transform.transform.translation.y
#     #             z = transform.transform.translation.z
#     #             self.get_logger().info(f"Received pose with TF sync: x={x}, y={y}")
#     #     except tf2_ros.LookupException as e:
#     #         self.get_logger().warn("Failed")


#     def publish_pose(self):
#         try:
#             transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
#             pose_msg = PoseStamped()
#             pose_msg.header.stamp = self.get_clock().now().to_msg()
#             pose_msg.header.frame_id = 'map'
#             pose_msg.pose.position.x = transform.transform.translation.x
#             pose_msg.pose.position.y = transform.transform.translation.y
#             pose_msg.pose.position.z = transform.transform.translation.z
#             pose_msg.pose.orientation = transform.transform.rotation
#             self.pose_pub.publish(pose_msg)
#             self.get_logger().info(f"Published a pose: x={pose_msg.pose.position.x}, y={pose_msg.pose.position.y}")
#         except tf2_ros.LookupException:
#             pass
    
# def main(args=None):
#     rclpy.init(args=args)
#     pose_node = PoseListenerNode()
#     try:
#         rclpy.spin(pose_node)
#     except KeyboardInterrupt:
#         pass

# if __name__ == '__main__':
#     main()
