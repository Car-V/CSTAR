import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# Define motor control pins
LEFT_FORWARD_PIN = 7
LEFT_BACKWARD_PIN = 8
RIGHT_FORWARD_PIN = 17
RIGHT_BACKWARD_PIN = 27

# Define enable pins
LEFT_ENABLE_PIN = 13
RIGHT_ENABLE_PIN = 12

PWM_MOTOR_MIN = 10  # Adjust based on testing
PWM_MOTOR_MAX = 75  # Full speed

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(LEFT_FORWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_FORWARD_PIN, GPIO.OUT)
GPIO.setup(RIGHT_BACKWARD_PIN, GPIO.OUT)
GPIO.setup(LEFT_ENABLE_PIN, GPIO.OUT)
GPIO.setup(RIGHT_ENABLE_PIN, GPIO.OUT)

left_forward_pwm = GPIO.PWM(LEFT_FORWARD_PIN, 100)
left_backward_pwm = GPIO.PWM(LEFT_BACKWARD_PIN, 100)
right_forward_pwm = GPIO.PWM(RIGHT_FORWARD_PIN, 100)
right_backward_pwm = GPIO.PWM(RIGHT_BACKWARD_PIN, 100)

left_forward_pwm.start(0)
left_backward_pwm.start(0)
right_forward_pwm.start(0)
right_backward_pwm.start(0)

# Enable motors by default
GPIO.output(LEFT_ENABLE_PIN, GPIO.HIGH)
GPIO.output(RIGHT_ENABLE_PIN, GPIO.HIGH)

class WheelController(Node):
    def __init__(self):
        super().__init__('wheel_controller')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def cmd_vel_callback(self, msg):
        linear_x = max(-1, min(1, msg.linear.x))  # Constrain values
        angular_z = max(-1, min(1, msg.angular.z))

        left_speed = (linear_x - angular_z) / 2.0
        right_speed = (linear_x + angular_z) / 2.0

        pwm_left = max(PWM_MOTOR_MIN, min(PWM_MOTOR_MAX, abs(left_speed) * 100))
        pwm_right = max(PWM_MOTOR_MIN, min(PWM_MOTOR_MAX, abs(right_speed) * 100))

        left_forward_pwm.ChangeDutyCycle(.905*pwm_left if left_speed > 0 else 0)
        left_backward_pwm.ChangeDutyCycle(.905*pwm_left if left_speed < 0 else 0)
        right_forward_pwm.ChangeDutyCycle(pwm_right if right_speed > 0 else 0)
        right_backward_pwm.ChangeDutyCycle(pwm_right if right_speed < 0 else 0)

        self.get_logger().info(f'Left PWM: {pwm_left}, Right PWM: {pwm_right}')

def main(args=None):
    rclpy.init(args=args)
    node = WheelController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        GPIO.cleanup()  # Clean up GPIO settings

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# import RPi.GPIO as GPIO
# import time

# # Motor & Encoder Pins (Update for your setup)
# LEFT_IN1, LEFT_IN2, RIGHT_IN1, RIGHT_IN2 = 7, 8, 17, 27
# LEFT_ENA, RIGHT_ENB = 13, 12  # PWM
# LEFT_ENCODER_A, LEFT_ENCODER_B = 5, 16  # Encoder input
# RIGHT_ENCODER_A, RIGHT_ENCODER_B = 2, 3 

# # class PIDController:
# #     def __init__(self, Kp, Ki, Kd):
# #         self.Kp = Kp
# #         self.Ki = Ki
# #         self.Kd = Kd
# #         self.last_error = 0
# #         self.integral = 0

# #     def compute(self, target, actual):
# #         error = target - actual
# #         self.integral += error
# #         derivative = error - self.last_error
# #         self.last_error = error
        
# #         return self.Kp * error + self.Ki * self.integral + self.Kd * derivative

# class MotorController(Node):
#     def __init__(self):
#         super().__init__('motor_controller')

#         self.subscription = self.create_subscription(
#             Twist, 'cmd_vel', self.cmd_vel_callback, 10
#         )

#         # Setup GPIO
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setup([LEFT_IN1, LEFT_IN2, LEFT_ENA, RIGHT_IN1, RIGHT_IN2, RIGHT_ENB], GPIO.OUT)
#         #GPIO.setup([LEFT_ENCODER_A, LEFT_ENCODER_B, RIGHT_ENCODER_A, RIGHT_ENCODER_B], GPIO.IN)

#         self.left_pwm = GPIO.PWM(LEFT_ENA, 100)
#         self.right_pwm = GPIO.PWM(RIGHT_ENB, 100)
#         self.left_pwm.start(15)
#         self.right_pwm.start(23)

#         # PID for position control
#         # self.left_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.05)
#         # self.right_pid = PIDController(Kp=0.5, Ki=0.01, Kd=0.05)

#         # self.left_ticks = 0
#         # self.right_ticks = 0
#         # self.left_velocity = 0.0
#         # self.right_velocity = 0.0
#         # self.last_time = time.time()

#         # Encoder Interrupts
#         # GPIO.add_event_detect(LEFT_ENCODER_A, GPIO.RISING, callback=self.left_encoder_callback)
#         # GPIO.add_event_detect(RIGHT_ENCODER_A, GPIO.RISING, callback=self.right_encoder_callback)

#     # def left_encoder_callback(self, channel):
#     #     self.left_ticks += 1
#     #     self.update_velocity()
#         # a_state = GPIO.input(LEFT_ENCODER_A)
#         # b_state = GPIO.input(LEFT_ENCODER_B)

#         # if a_state != self.left_last_state:
#         #     if b_state == a_state:
#         #         self.left_ticks += 1
#         #     else:
#         #         self.left_ticks -= 1
#         # self.left_last_state = a_state

#     # def right_encoder_callback(self, channel):
#     #     self.right_ticks += 1
#     #     self.update_velocity()
#         # a_state = GPIO.input(RIGHT_ENCODER_A)
#         # b_state = GPIO.input(RIGHT_ENCODER_B)

#         # if a_state != self.right_last_state:
#         #     if b_state == a_state:
#         #         self.right_ticks += 1
#         #     else:
#         #         self.right_ticks -= 1
#         # self.right_last_state = a_state

#     def update_velocity(self):
#         current_time = time.time()
#         time_elapsed = current_time - self.last_time
#         if time_elapsed > 0.1:
#             self.left_velocity = self.left_ticks / time_elapsed
#             self.right_velocity = self.right_ticks / time_elapsed
#             self.left_ticks = 0
#             self.right_ticks = 0
#             self.last_time = current_time

#     def cmd_vel_callback(self, msg):
#         target_velocity = msg.linear.x  # Scale appropriately
#         left_output = self.left_pid.compute(target_velocity, self.left_velocity)
#         right_output = self.right_pid.compute(target_velocity, self.right_velocity)

#         # Set motor direction
#         GPIO.output(LEFT_IN1, GPIO.HIGH if left_output > 0 else GPIO.LOW)
#         GPIO.output(LEFT_IN2, GPIO.LOW if left_output > 0 else GPIO.HIGH)
#         GPIO.output(RIGHT_IN1, GPIO.HIGH if right_output > 0 else GPIO.LOW)
#         GPIO.output(RIGHT_IN2, GPIO.LOW if right_output > 0 else GPIO.HIGH)

#         if abs(msg.linear.x) < 0.01 and abs(msg.angular.z) < 0.01:
#             self.left_pwm.ChangeDutyCycle(0)
#             self.right_pwm.ChangeDutyCycle(0)
#         else:
#             correction = self.left_velocity - self.right_velocity
#             self.left_pwm.ChangeDutyCycle(max(0, 20 - correction))
#             self.right_pwm.ChangeDutyCycle(max(0, 20 + correction))

#         # # Set PWM speed (clamped to valid range)
#         # self.left_pwm.ChangeDutyCycle(min(abs(left_output), 100))
#         # self.right_pwm.ChangeDutyCycle(min(abs(right_output), 100))

#     def cleanup(self):
#         self.left_pwm.stop()
#         self.right_pwm.stop()
#         GPIO.cleanup()
#         self.get_logger().info("Motor control shut down cleanly")

# def main(args=None):
#     rclpy.init(args=args)
#     motor_controller = MotorController()
#     try:
#         rclpy.spin(motor_controller)
#     except KeyboardInterrupt:
#         motor_controller.cleanup()
#     finally:
#         motor_controller.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
