import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from std_msgs.msg import Header
import tf_transformations
import math

class EncoderOdometry(Node):

    def __init__(self):
        super().__init__('encoder_odometry')
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.publish_odometry)
        self.encoder_left = 0
        self.encoder_right = 0

    def publish_odometry(self):
        # Fetch encoder values and calculate odometry
        # This is just a dummy example; replace with actual calculations
        x, y, theta = self.calculate_odometry(self.encoder_left, self.encoder_right)

        # Create the odometry message
        odom_msg = Odometry()

        # Header
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'

        # Pose
        pose = PoseWithCovariance()
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        odom_msg.pose = pose

        # Twist
        twist = TwistWithCovariance()
        twist.twist.linear.x = 0.1  # Replace with actual linear velocity
        twist.twist.angular.z = 0.1  # Replace with actual angular velocity
        odom_msg.twist = twist

        # Publish the message
        self.odom_publisher.publish(odom_msg)

    def calculate_odometry(self, encoder_left, encoder_right):
        # Replace with actual calculations using encoder values
        x = 0.0
        y = 0.0
        theta = 0.0
        return x, y, theta

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()