import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time
import math

# Motor & Encoder Pins (Update for your setup)
LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2 = 23, 24, 27, 22
LEFT_ENA, RIGHT_ENB = 18, 19  # PWM
LEFT_ENCODER, RIGHT_ENCODER = 17, 16  # Encoder input

# Wheel & Robot Constants
WHEEL_SEPARATION = 0.3  # Distance between wheels (meters)
Y_ADJUST_GAIN = 2.0  # Tunable gain for converting linear.y to rotation
Y_THRESHOLD = 0.01  # Minimum Y command to trigger rotation

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.last_error = 0
        self.integral = 0

    def compute(self, target, actual):
        error = target - actual
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([LEFT_IN1, LEFT_IN2, LEFT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB], GPIO.OUT)
        GPIO.setup([LEFT_ENCODER, RIGHT_ENCODER], GPIO.IN)

        self.left_pwm = GPIO.PWM(LEFT_ENA, 1000)
        self.right_pwm = GPIO.PWM(RIGHT_ENB, 1000)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

        # PID for position control
        self.left_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.05)
        self.right_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.05)

        self.left_ticks = 0
        self.right_ticks = 0

        # Encoder Interrupts
        GPIO.add_event_detect(LEFT_ENCODER, GPIO.RISING, callback=self.left_encoder_callback)
        GPIO.add_event_detect(RIGHT_ENCODER, GPIO.RISING, callback=self.right_encoder_callback)

    def left_encoder_callback(self, channel):
        self.left_ticks += 1

    def right_encoder_callback(self, channel):
        self.right_ticks += 1

    def cmd_vel_callback(self, msg):
        # Extract velocities
        target_linear_x = msg.linear.x * 100  # Scale for forward/backward
        target_linear_y = msg.linear.y * 100  # Y-direction (converted to rotation)
        target_angular_z = msg.angular.z * 50  # Scale for turning

        # Convert Y movement into rotation if necessary
        if abs(target_linear_y) > Y_THRESHOLD:
            desired_angle = math.atan2(target_linear_y, target_linear_x)  # Target heading
            target_angular_z += Y_ADJUST_GAIN * desired_angle  # Adjust rotation

        # Compute wheel targets for differential drive
        left_target = target_linear_x - (target_angular_z * WHEEL_SEPARATION / 2)
        right_target = target_linear_x + (target_angular_z * WHEEL_SEPARATION / 2)

        # PID control for each motor
        left_output = self.left_pid.compute(left_target, self.left_ticks)
        right_output = self.right_pid.compute(right_target, self.right_ticks)

        # Set motor directions
        GPIO.output(LEFT_IN1, GPIO.HIGH if left_output > 0 else GPIO.LOW)
        GPIO.output(LEFT_IN2, GPIO.LOW if left_output > 0 else GPIO.HIGH)
        GPIO.output(RIGHT_IN1, GPIO.HIGH if right_output > 0 else GPIO.LOW)
        GPIO.output(RIGHT_IN2, GPIO.LOW if right_output > 0 else GPIO.HIGH)

        # Adjust PWM speed (clamped to valid range)
        self.left_pwm.ChangeDutyCycle(min(abs(left_output), 100))
        self.right_pwm.ChangeDutyCycle(min(abs(right_output), 100))

    def cleanup(self):
        self.left_pwm.stop()
        self.right_pwm.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        motor_controller.cleanup()
    finally:
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()