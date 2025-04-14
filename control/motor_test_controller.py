# Use DualSense controller connected via Bluetooth

import RPi.GPIO as GPIO
from time import sleep
from pydualsense import pydualsense

GPIO.setwarnings(False)

# Right Motor
in1 = 17
in2 = 27
en_a = 12
# Left Motor
in3 = 8
in4 = 7
en_b = 13

DEAD_ZONE = 10

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en_a,GPIO.OUT)

GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(en_b,GPIO.OUT)

pwm_speed = 75
q=GPIO.PWM(en_a,pwm_speed)
p=GPIO.PWM(en_b,pwm_speed)
p.start(20.5)
q.start(20) #26
GPIO.output([in1, in2, in3, in4], GPIO.LOW)
ds = pydualsense()
ds.init()
ds.state.LX = 0.0
ds.state.LY = 0.0

def set_motor(left, right):
   GPIO.output(in1, GPIO.HIGH if right > 0 else GPIO.LOW)
   GPIO.output(in2, GPIO.HIGH if right < 0 else GPIO.LOW)
   GPIO.output(in3, GPIO.HIGH if left < 0 else GPIO.LOW)
   GPIO.output(in4, GPIO.HIGH if left > 0 else GPIO.LOW)

def joystick_movement():
   x = ds.state.LX
   y = ds.state.LY
   print(f"Joystick - LX: {x}, LY: {y}")
   if abs(x) < DEAD_ZONE and abs(y) < DEAD_ZONE:
      print("Dead zone triggered, stop motors")
      set_motor(0, 0)
   else:
      left_speed = -y + x
      right_speed = -y - x
      print(f"Motor speeds - Left: {left_speed}, Right: {right_speed}")
      set_motor(left_speed, right_speed)

#ds.LX_changed += lambda x: joystick_movement(x, ds.LY)
#ds.LY_changed += lambda y: joystick_movement(ds.Lx, y)

try:
   while True:
      joystick_movement()
      sleep(0.1)
except KeyboardInterrupt:
   ds.close()
   GPIO.cleanup()
   print("exiting, GPIO cleanup")
