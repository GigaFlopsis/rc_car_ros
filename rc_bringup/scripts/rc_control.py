#! /usr/bin/env python
# coding: utf-8
"""
This is Ros node for pwm control rc car
"""

import RPi.GPIO as GPIO
import pigpio
import time
import numpy as np

import rospy
from geometry_msgs.msg import Twist, Vector3


# init params

servo_pin = 4 # inut pin of servo
motor_pin = 17 # inut pin of motor

middle_servo = 1500
middle_motor = 1550
offset = 47.0 # offset of servo

motor_power = 0.2 # limit the power of the motor (1.0 max)
cmd_vel_topic = "rc_car/cmd_vel" # output topic

pi = pigpio.pi()
pi.set_servo_pulsewidth(servo_pin, middle_servo) # middle servo angle
pi.set_servo_pulsewidth(motor_pin, middle_motor) # zero speed for motor (different depending on ESC)

vel_msg = Twist()

rate = 5
time_clb = 0.0

def get_angle_to_mills(angle):
    """
    calculate angle degres to pulse
    """
    return (angle / 18) + 2.5

def vel_clb(data):
    """
    Get velocity value from topic
    :param data: velocity value
    :type data: Twist

    """
    global  vel_msg, time_clb
    vel_msg = data
    vel_msg.angular.x = np.clip(vel_msg.angular.z, -1.0, 1.0)
    vel_msg.linear.x = np.clip(vel_msg.linear.x-vel_msg.linear.y, -1.0, 1.0)
    set_rc_remote()

    time_clb = 0.0

def set_rc_remote():
    """
    Recalculation velocity data to pulse and set to PWM servo and motor
    :return:
    """
    global vel_msg, motor_power
    # send servo
    servo_val = valmap(vel_msg.angular.z, 1, -1, 1000+offset, 2000+offset)
    pi.set_servo_pulsewidth(servo_pin, servo_val)
    # send motor
    motor_val = valmap(vel_msg.linear.x, -1.0/motor_power, 1.0/motor_power, 1050, 2050)
    pi.set_servo_pulsewidth(motor_pin, motor_val)

def valmap(value, istart, istop, ostart, ostop):
    """
    Re-maps a number from one range to another.
    That is, a value of istart would get mapped to ostart,
    a value of istop to ostop, values in-between to values in-between, etc.
    :param value: value
    :param istart:  the lower bound of the value’s current range
    :param istop: the upper bound of the value’s current range
    :param ostart: the lower bound of the value’s target range
    :param ostop: the upper bound of the value’s target range
    :return: The mapped value.
    """
    val = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
    return np.clip(val, ostart, ostop)


if __name__ == "__main__":
    try:
        rospy.init_node("rc_control")
        rate = rospy.Rate(rate)

        # get args from ros params
        name_node = rospy.get_name();
        cmd_vel_topic = rospy.get_param(name_node + '/cmd_vel', cmd_vel_topic);
        servo_pin = rospy.get_param(name_node + '/servo_pin', servo_pin);
        middle_servo = rospy.get_param(name_node + '/middle_servo', middle_servo);
        offset = rospy.get_param(name_node + '/servo_offset', offset);
        motor_pin = rospy.get_param(name_node + '/motor_pin', motor_pin);
        middle_motor = rospy.get_param(name_node + '/middle_motor', middle_motor);
        motor_power = rospy.get_param(name_node + '/motor_power', motor_power);


        rospy.Subscriber(cmd_vel_topic, Twist, vel_clb)

        print ("RC_control params:"
               "cmd_vel: %s \n"
               "servo_pin: %d \n"
               "middle_servo: %d \n"
               "servo_offset: %d \n"
               "motor_pin: %d \n"
               "middle_motor: %d \n"
               "motor_power: %d \n" % (cmd_vel_topic, servo_pin,middle_servo,offset,motor_pin,middle_motor,motor_power))
        while not rospy.is_shutdown():
            try:
                time_clb += 0.2
                if(time_clb > 1.0):     # if data does not come to close pwm
                    vel_msg = Twist()
                    set_rc_remote()
            except:
                time_clb += 0.2
                if(time_clb > 1.0):     # if something is wrong to close pwm
                    vel_msg = Twist()
                    set_rc_remote()
		    motor.stop()
                print("error")
            rate.sleep()

    except KeyboardInterrupt:   # if put ctr+c
        print("ctrl+C exit")
        pi.set_servo_pulsewidth(servo_pin, 0)
        pi.set_servo_pulsewidth(motor_pin, 0)
        pi.stop()
        GPIO.cleanup()
    finally: # if exit
        print("exit")
        pi.set_servo_pulsewidth(servo_pin, 0)
        pi.set_servo_pulsewidth(motor_pin, 0)
        pi.stop()
        GPIO.cleanup()
