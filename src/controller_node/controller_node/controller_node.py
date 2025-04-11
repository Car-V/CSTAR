# controller_node, attempted to integrate Bluetooth control program into node for cstar_launch_controller launch
# Code by Carina Vale

import RPi.GPIO as GPIO
from pydualsense import pydualsense
import rclpy
from rclpy.node import Node

# Right Motor
in1 = 17
in2 = 27
en_a = 12
# Left Motor
in3 = 8
in4 = 7
en_b = 13

DEAD_ZONE = 10

class ControllerNode(Node):
   def __init__(self):
      super().__init__('controller_node')
      self.ds = pydualsense()
      self.timer = self.create_timer(0.1, self.joystick_movement)

      self.start()

   def start(self):
      
      GPIO.setwarnings(False)
      GPIO.setmode(GPIO.BCM)
      GPIO.setup([in1, in2, en_a, in3, in4, en_b], GPIO.OUT)

      pwm_speed = 75
      q=GPIO.PWM(en_a,pwm_speed)
      p=GPIO.PWM(en_b,pwm_speed)
      p.start(19)
      q.start(48)
      GPIO.output([in1, in2, in3, in4], GPIO.LOW)
      self.ds.init()
      self.ds.state.LX = 0.0
      self.ds.state.LY = 0.0

   def set_motor(self, left, right):
      GPIO.output(in1, GPIO.HIGH if right > 0 else GPIO.LOW)
      GPIO.output(in2, GPIO.HIGH if right < 0 else GPIO.LOW)
      GPIO.output(in3, GPIO.HIGH if left < 0 else GPIO.LOW)
      GPIO.output(in4, GPIO.HIGH if left > 0 else GPIO.LOW)

   def joystick_movement(self):
      x = self.ds.state.LX
      y = self.ds.state.LY
      print(f"Joystick - LX: {x}, LY: {y}")
      if abs(x) < DEAD_ZONE and abs(y) < DEAD_ZONE:
         print("Dead zone triggered, stop motors")
         self.set_motor(0, 0)
      else:
         left_speed = -y + x
         right_speed = -y - x
         print(f"Motor speeds - Left: {left_speed}, Right: {right_speed}")
         self.set_motor(left_speed, right_speed)

   def destroy_node(self):
      self.ds.close()
      GPIO.cleanup()
      print("exiting, GPIO cleanup")
      super().destroy_node()


#ds.LX_changed += lambda x: joystick_movement(x, ds.LY)
#ds.LY_changed += lambda y: joystick_movement(ds.Lx, y)



def main(args=None):
   rclpy.init(args=args)
   controller_node = ControllerNode()
   try:
      rclpy.spin(controller_node)
   except KeyboardInterrupt:
      controller_node.destroy_node()
      rclpy.shutdown()

if __name__=='__main__':
   main()
