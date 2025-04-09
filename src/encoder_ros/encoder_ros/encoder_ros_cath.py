from math import sin, cos, pi
import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

# Hardware Constants
WHEEL_DIAMETER = 0.072        # meters
PULSES_PER_REV = 537.7        # encoder pulses per revolution (check if gear ratio is included)
GEAR_RATIO = 19.2             # if your encoder spec is at the motor; adjust as needed
WHEEL_BASE = 0.25             # distance between wheels (meters)
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER
# If the encoder spec already accounts for the gear, you might omit the GEAR_RATIO factor.
DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (PULSES_PER_REV * GEAR_RATIO)

# GPIO Pins for the quadrature encoders
LEFT_A = 5
LEFT_B = 16
RIGHT_A = 2
RIGHT_B = 3

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        # Create publisher and tf broadcaster
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.odom_broadcaster = TransformBroadcaster(self)

        # Robot pose variables (in world frame)
        self.x = 0.0   # meters
        self.y = 0.0   # meters
        self.th = 0.0  # radians (heading)

        # Encoder tick counters
        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        # Initialize time stamps
        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()

        # Set up GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEFT_A, GPIO.IN)
        GPIO.setup(LEFT_B, GPIO.IN)
        GPIO.setup(RIGHT_A, GPIO.IN)
        GPIO.setup(RIGHT_B, GPIO.IN)

        # Save the initial state for channel A for each encoder (for quadrature decoding)
        self.last_left_A_state = GPIO.input(LEFT_A)
        self.last_right_A_state = GPIO.input(RIGHT_A)

        # Attach interrupts on channel A for both encoders (BOTH edges for more robust decoding)
        GPIO.add_event_detect(LEFT_A, GPIO.BOTH, callback=self.left_encoder_callback, bouncetime=1)
        GPIO.add_event_detect(RIGHT_A, GPIO.BOTH, callback=self.right_encoder_callback, bouncetime=1)

        # Create a timer callback at 10Hz to update odometry
        self.timer = self.create_timer(0.1, self.timer_callback)

    def left_encoder_callback(self, channel):
        """Callback for left encoder channel A.
        Reads channel B to determine rotation direction."""
        current_A = GPIO.input(LEFT_A)
        current_B = GPIO.input(LEFT_B)
        if current_A != self.last_left_A_state:
            # Typical quadrature decoding:
            # If channel A equals channel B, assume forward rotation; else reverse.
            if current_A == current_B:
                self.left_ticks += 1
            else:
                self.left_ticks -= 1
        self.last_left_A_state = current_A

    def right_encoder_callback(self, channel):
        """Callback for right encoder channel A.
        Reads channel B to determine rotation direction."""
        current_A = GPIO.input(RIGHT_A)
        current_B = GPIO.input(RIGHT_B)
        if current_A != self.last_right_A_state:
            if current_A == current_B:
                self.right_ticks += 1
            else:
                self.right_ticks -= 1
        self.last_right_A_state = current_A

    def calculate_delta(self, dt):
        """Compute the incremental distance traveled by each wheel and return the average distance and heading change."""
        d_left = (self.left_ticks - self.last_left_ticks) * DISTANCE_PER_PULSE
        d_right = (self.right_ticks - self.last_right_ticks) * DISTANCE_PER_PULSE

        # Update last tick counts for next cycle
        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

        d_center = (d_left + d_right) / 2.0
        d_th = (d_right - d_left) / WHEEL_BASE
        return d_center, d_th

    def timer_callback(self):
        """Called periodically to update the robot pose and publish odometry."""
        self.current_time = self.get_clock().now()
        dt = (self.current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        d_center, d_th = self.calculate_delta(dt)

        # Compute change in position using exact integration over a circular arc
        if abs(d_th) < 1e-6:
            # Straight-line motion approximation
            delta_x = d_center * cos(self.th)
            delta_y = d_center * sin(self.th)
        else:
            # Robot follows an arc of a circle
            delta_x = (d_center / d_th) * (sin(self.th + d_th) - sin(self.th))
            delta_y = (d_center / d_th) * (-cos(self.th + d_th) + cos(self.th))

        # Update pose
        self.x += delta_x
        self.y += delta_y
        self.th = (self.th + d_th) % (2 * pi)

        # Compute velocities
        vx = d_center / dt
        vth = d_th / dt

        # Create a quaternion from yaw (only z-axis rotation)
        odom_quat = Quaternion()
        odom_quat.z = sin(self.th / 2.0)
        odom_quat.w = cos(self.th / 2.0)

        # Publish the transform over tf2
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat
        self.odom_broadcaster.sendTransform(odom_trans)

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        self.odom_pub.publish(odom)

        self.last_time = self.current_time

def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
