#! /usr/bin/env python
# coding: utf-8

import rospy
from geometry_msgs.msg import Twist, Vector3

import RPi.GPIO as GPIO
import time
import numpy as np

# set GPIO params
GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)
GPIO.setup(11,GPIO.OUT)
servo = GPIO.PWM(7,50)
motor = GPIO.PWM(11,50)
servo.start(7.5)
motor.start(7.5)

offset = -5.0

motor_power = 0.1

vel_msg = Twist()

rate = 5
time_clb = 0.0

def get_angle_to_mills(angle):
    return (angle / 18) + 2.5

def vel_clb(data):
    global  vel_msg, time_clb
    vel_msg = data
    vel_msg.angular.x = np.clip(vel_msg.angular.z, -1.0, 1.0)
    vel_msg.linear.x = np.clip(vel_msg.linear.x-vel_msg.linear.y, -1.0, 1.0)
    set_rc_remote()
#    print(vel_msg)
    time_clb = 0.0

def set_rc_remote():
    global vel_msg, motor_power
    servo_val = valmap(vel_msg.angular.z, 1, -1, 0, 180)
    servo.ChangeDutyCycle(get_angle_to_mills(servo_val+offset))
    motor_val = valmap(vel_msg.linear.x, -1.0/motor_power, 1.0/motor_power, 3.4, 11.4)
    motor.ChangeDutyCycle(motor_val)

def valmap(value, istart, istop, ostart, ostop):
    val = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
    return np.clip(val, ostart, ostop)



if __name__ == "__main__":
    try:
        rospy.init_node("simple_marker")
        rate = rospy.Rate(rate)
        rospy.Subscriber("rc_car/cmd_vel", Twist, vel_clb)

        while not rospy.is_shutdown():
            try:
                time_clb += 0.2
                if(time_clb > 1.0):
                    print ("time_clb > 1")
                    vel_msg = Twist()
                    set_rc_remote()
            except:
                time_clb += 0.2
                if(time_clb > 1.0):
                    print ("time_clb > 1")
                    vel_msg = Twist()
                    set_rc_remote()
		    motor.stop()
                print("error")
            rate.sleep()

    except KeyboardInterrupt:
        print("ctrl+C exit")
        motor.stop()
        servo.stop()
        GPIO.cleanup()
    finally:
        print("exit")
        motor.stop()
        servo.stop()
        GPIO.cleanup()
