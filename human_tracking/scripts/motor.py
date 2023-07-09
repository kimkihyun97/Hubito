#!/usr/bin/env python
import rospy
import Jetson.GPIO as GPIO
from std_msgs.msg import Float32MultiArray

GPIO.setmode(GPIO.BOARD)

PWM1 = 32
PWM2 = 33
DIR1 = 37
DIR2 = 18

# Setup GPIO channels
GPIO.setup(PWM1, GPIO.OUT)
GPIO.setup(PWM2, GPIO.OUT)
GPIO.setup(DIR1, GPIO.OUT)
GPIO.setup(DIR2, GPIO.OUT)

# Setup PWM channels
pwm1 = GPIO.PWM(PWM1, 1000)
pwm2 = GPIO.PWM(PWM2, 1000)

pwm1.start(0)
pwm2.start(0)

def control_callback(control_pwm):
    if len(control_pwm.data) < 2:
        rospy.logerr("Received control PWM does not contain 2 elements")
        return
    left_pwm = control_pwm.data[0]
    right_pwm = control_pwm.data[1]

    # Set directions
    GPIO.output(DIR1, GPIO.HIGH if left_pwm >= 0 else GPIO.LOW)
    GPIO.output(DIR2, GPIO.HIGH if right_pwm >= 0 else GPIO.LOW)

    # Set PWM
    pwm1.ChangeDutyCycle(abs(left_pwm))
    pwm2.ChangeDutyCycle(abs(right_pwm))

def end_motors():
   GPIO.output(DIR1, GPIO.LOW)
   GPIO.output(DIR2, GPIO.LOW)
   p1.start(0)
   p2.start(0)

def listener():
    rospy.init_node('motor_driver')
    rospy.Subscriber('control_command', Float32MultiArray, control_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    end_motors()
    GPIO.cleanup()


