from math import sin, cos, pi
import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion

WHEEL_DIAMETER = 0.072  
PULSES_PER_REV = 537.7  
GEAR_RATIO = 19.2       
WHEEL_BASE = 0.25 
WHEEL_CIRCUMFERENCE = pi * WHEEL_DIAMETER
DISTANCE_PER_PULSE = WHEEL_CIRCUMFERENCE / (PULSES_PER_REV * GEAR_RATIO)
LEFT_A = 5 # update
LEFT_B = 16
RIGHT_A = 2
RIGHT_B = 3

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)       
        self.odom_broadcaster = TransformBroadcaster(self)
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.left_ticks = 0
        self.right_ticks = 0
        self.last_left_ticks = 0
        self.last_right_ticks = 0

        #to run without GPIO, uncomment
        #self.vx = 0.1
        #self.vy = -0.1
        #self.vth = 0.1

        self.current_time = self.get_clock().now()
        self.last_time = self.get_clock().now()
       
        # to run without GPIO, comment out GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LEFT_A, GPIO.IN)
        GPIO.setup(LEFT_B, GPIO.IN)
        GPIO.setup(RIGHT_A, GPIO.IN)
        GPIO.setup(RIGHT_B, GPIO.IN)
        GPIO.add_event_detect(LEFT_A, GPIO.RISING, callback=self.increment_left_ticks) # check these are the right rising
        GPIO.add_event_detect(RIGHT_A, GPIO.RISING, callback=self.increment_left_ticks)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def increment_left_ticks(self, channel):
        self.left_ticks += 1
    
    def increment_right_ticks(self, channel):
        self.right_ticks += 1

    def calculate_velocities(self):
        elapsed_time = (self.current_time - self.last_time).nanoseconds / 1e9

        left_distance = (self.left_ticks - self.last_left_ticks) * DISTANCE_PER_PULSE
        right_distance = (self.right_ticks - self.last_right_ticks) * DISTANCE_PER_PULSE

        self.last_left_ticks = self.left_ticks
        self.last_right_ticks = self.right_ticks

        linear_velocity = (left_distance + right_distance) / (2.0 * elapsed_time)
        angular_velocity = (right_distance - left_distance) / (WHEEL_BASE * elapsed_time)

        return linear_velocity, angular_velocity

    def timer_callback(self):
       
        self.current_time = self.get_clock().now()

        # to run without GPIO, comment out
        vx, vth = self.calculate_velocities()

        # compute odometry given velocities
        dt = (self.current_time - self.last_time).nanoseconds / 1e9

        delta_x = vx * cos(self.th) * dt
        delta_y = vx * sin(self.th) * dt
        delta_th = vth * dt
        # to run without GPIO, switch with
        #delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        #delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        #delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # create Quaternion from yaw / z-axis
        odom_quat = Quaternion()
        odom_quat.z = sin(self.th / 2.0)
        odom_quat.w = cos(self.th / 2.0)

        # publish transform over tf
        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.current_time.to_msg()
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id = 'base_link'
        
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat
        
        # send transform
        self.odom_broadcaster.sendTransform(odom_trans)
        
        # publish odometry over ros
        odom = Odometry()
        odom.header.stamp = self.current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
                
        # set velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth
        # to run without GPIO, switch
        #odom.twist.twist.linear.x = self.vx
        #odom.twist.twist.linear.y = self.vy
        #odom.twist.twist.angular.z = self.vth
        
        # publish message
        self.odom_pub.publish(odom)
        self.last_time = self.current_time
             

def main(args=None):
    rclpy.init(args=args)
    
    encoder_odometry_node = EncoderOdometryNode()
    
    try:
        rclpy.spin(encoder_odometry_node)
    
    except KeyboardInterrupt:
        pass
    

    encoder_odometry_node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
