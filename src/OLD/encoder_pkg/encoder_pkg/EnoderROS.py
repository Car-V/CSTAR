import math
import time
import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

WHEEL_DIAMETER = 0.072  
PULSES_PER_REV = 537.7  
GEAR_RATIO = 19.2
WHEEL_BASE = 0.25 

class Encoder:
   

    def __init__(self, pin_a: int, pin_b: int):
        self.pin_a = pin_a
        self.pin_b = pin_b
        self.position = 0
        self.direction = 0  # 1 for clockwise, -1 for counterclockwise
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_a, GPIO.IN)
        GPIO.setup(self.pin_b, GPIO.IN)
        
        self.last_a_state = GPIO.input(self.pin_a)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()

    def update_position(self):
        # Read the state of the encoder channel A
        current_a_state = GPIO.input(self.pin_a)
        current_b_state = GPIO.input(self.pin_b)

        # Check if the A channel changed
        if current_a_state != self.last_a_state:
            # If the A channel leads the B channel, it's a clockwise rotation
            if current_b_state != current_a_state:
                self.position += 1
                self.direction = 1  # Clockwise
            else:
                self.position -= 1
                self.direction = -1  # Counterclockwise

        self.last_a_state = current_a_state  # Update the last state for next comparison

    def get_position(self):
        return self.position

    def get_direction(self):
        return self.direction

    def reset_position(self):
        self.position = 0

    def get_odometry(self):
        return self.x, self.y, self.theta


class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')

        self.left_encoder = Encoder(pin_a=17, pin_b=18)  # NEED TO UPDATE PIN
        self.right_encoder = Encoder(pin_a=22, pin_b=23)  # NEED TO UPDATE PIN

        self.odometry_publisher = self.create_publisher(Float32MultiArray, 'odom', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def calculate_distance(self, encoder: Encoder):
        wheel_circumference = math.pi * WHEEL_DIAMETER
        distance_per_pulse = wheel_circumference / (PULSES_PER_REV * GEAR_RATIO)
        return encoder.position * distance_per_pulse

    def update_odometry(self, left_encoder: Encoder, right_encoder: Encoder):
        left_distance = self.calculate_distance(left_encoder)
        right_distance = self.calculate_distance(right_encoder)
        
        distance = (right_distance + left_distance) / 2.0
        delta_theta = (right_distance - left_distance) / WHEEL_BASE
        
        left_encoder.theta += delta_theta
        left_encoder.x += distance * math.cos(left_encoder.theta)
        left_encoder.y += distance * math.sin(left_encoder.theta)
        right_encoder.theta += delta_theta
        right_encoder.x += distance * math.cos(right_encoder.theta)
        right_encoder.y += distance * math.sin(right_encoder.theta)

    def timer_callback(self):
        self.left_encoder.update_position()
        self.right_encoder.update_position()

        self.update_odometry(self.left_encoder, self.right_encoder)

        odometry = Float32MultiArray()
        odometry.data = [self.left_encoder.x, self.left_encoder.y, self.left_encoder.theta]
        
        self.odometry_publisher.publish(odometry)

def main(args=None):
    rclpy.init(args=args)

    encoder_odometry_node = EncoderOdometryNode()

    rclpy.spin(encoder_odometry_node)

    encoder_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()