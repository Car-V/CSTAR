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

# GPIO.output(in1,GPIO.LOW)
# GPIO.output(in2,GPIO.LOW)
# GPIO.output(in4,GPIO.LOW)
# GPIO.output(in3,GPIO.LOW)
# def run():
#    # Wrap main content in a try block so we can  catch the user pressing CTRL-C and run the
#    # GPIO cleanup function. This will also prevent the user seeing lots of unnecessary error messages.
#    try:
#    # Create Infinite loop to read user input
#       while(True):
#          # Get user Input
#          user_input = input()

#          # To see users input
#          # print(user_input)

#          if user_input == 'w':
#             pause()
#             GPIO.output(in1,GPIO.HIGH)
#             GPIO.output(in2,GPIO.LOW)

#             GPIO.output(in4,GPIO.HIGH)
#             GPIO.output(in3,GPIO.LOW)
            

#             print("Forward")

#          elif user_input == 's':
#             pause()
#             GPIO.output(in1,GPIO.LOW)
#             GPIO.output(in2,GPIO.HIGH)

#             GPIO.output(in4,GPIO.LOW)
#             GPIO.output(in3,GPIO.HIGH)
            
#             print('Backward')

#          elif user_input == 'd':
#             pause()
#             GPIO.output(in1,GPIO.LOW)
#             GPIO.output(in2,GPIO.HIGH)

#             GPIO.output(in4,GPIO.LOW)
#             GPIO.output(in3,GPIO.LOW)
            
#             print('Right')

#          elif user_input == 'a':
#             pause()
#             GPIO.output(in1,GPIO.HIGH)
#             GPIO.output(in2,GPIO.LOW)

#             GPIO.output(in4,GPIO.LOW)
#             GPIO.output(in3,GPIO.LOW)
            
#             print('Left')

#          # Press 'c' to exit the script
#          elif user_input == 'c':
#             GPIO.output(in1,GPIO.LOW)
#             GPIO.output(in2,GPIO.LOW)

#             GPIO.output(in4,GPIO.LOW)
#             GPIO.output(in3,GPIO.LOW)
#             print('Stop')

#    # If user press CTRL-C
#    except KeyboardInterrupt:
#    # Reset GPIO settings
#       GPIO.cleanup()
#       print("GPIO Clean up")

# def pause():
#    GPIO.output(in1,GPIO.LOW)
#    GPIO.output(in2,GPIO.LOW)

#    GPIO.output(in4,GPIO.LOW)
#    GPIO.output(in3,GPIO.LOW)
#    sleep(0.5)


# # def slow_bringup():
# #    #increment speed
# #    for pwm_speed in range (0,51,2):
# #       q.ChangeDutyCycle(pwm_speed)
# #       p.ChangeDutyCycle(pwm_speed)
# #       sleep(0.6)

# # def slow_bringdown():
# #    #decrement speed
# #    for pwm_speed in range (50,-1,2):
# #       q.ChangeDutyCycle(pwm_speed)
# #       p.ChangeDutyCycle(pwm_speed)
# #       sleep(0.6)
      
#    GPIO.output(in1,GPIO.LOW)
#    GPIO.output(in2,GPIO.LOW)

#    GPIO.output(in4,GPIO.LOW)
#    GPIO.output(in3,GPIO.LOW)

# def main():
#    run()

# if __name__=='__main__':
#    main()
