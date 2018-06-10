#! /usr/bin/env python
# coding: utf-8
"""
This is Ros node for pwm control rc car
"""

import RPi.GPIO as GPIO
import pigpio
import time
import numpy as np
import math
from enum import Enum


import rospy
from geometry_msgs.msg import Twist
from rc_bringup.msg import CarPwmContol
from ackermann_msgs.msg import AckermannDriveStamped


class RemoteMode(Enum):
    vel = 0
    pwm = 1
    drive = 2


# init params

servo_pin = 4 # inut pin of servo
motor_pin = 17 # inut pin of motor

middle_servo = 1500
middle_motor = 1500
offset = 47.0 # offset of servo
revers_servo = False # revers of servo direction
revers_val = 1.0

max_vel = 1.0 # max speed of the car
min_vel = 0.7 # min speed of the car
max_steering_angle = 25.0 # in degrees
wheelbase = 0.28 # in meters

"""Topics for remote car"""
cmd_vel_topic = "/cmd_vel"       # remote via velocity
pwm_topic = "/pwm"               # direct remote PWM
drive_topic = "/ackermann_cmd"   # remote like-car
pwm_output_topic = "/pwm_output"   # remote like-car

intercept_remote = False

pi = pigpio.pi()
pi.set_servo_pulsewidth(servo_pin, middle_servo) # middle servo angle
pi.set_servo_pulsewidth(motor_pin, middle_motor) # zero speed for motor (different depending on ESC)

vel_msg = Twist()
pwm_msg = CarPwmContol()
drive_msg = AckermannDriveStamped()
pwm_output_msg = CarPwmContol()

rate = 5
time_clb = 0.0


def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  global max_steering_angle
  if omega == 0.0:
    return 0
  if v == 0.0:
      return math.degrees(omega)


  radius = v / omega
  steering_angle = math.degrees(math.atan(wheelbase / radius))
  return np.clip(steering_angle, -max_steering_angle, max_steering_angle)

def vel_clb(data):
    """
    Get velocity value from topic
    :param data: velocity value
    :type data: Twist

    """
    global vel_msg, time_clb, max_vel, min_vel
    vel_msg = data
    vel_msg.linear.x = np.clip(vel_msg.linear.x-vel_msg.linear.y, -max_vel, max_vel)
    if vel_msg.linear.x != 0.0:
        if abs(vel_msg.linear.x) < min_vel:
            vel_msg.linear.x = min_vel if vel_msg.linear.x > 0 else min_vel


    set_rc_remote(RemoteMode.vel)
    time_clb = 0.0

def vel_clb_pwm(data):
    """
    Get PWM value from topic
    :param data: velocity value
    :type data: RcCarControl

    """
    global pwm_msg, time_clb
    pwm_msg = data
    set_rc_remote(RemoteMode.pwm)
    time_clb = 0.0

def vel_clb_drive(data):
    """
       Get drive value from topic
       :param data: velocity and  steering value
       :type data: AckermannDriveStamped
       """
    global drive_msg,time_clb, max_vel
    drive_msg = data
    drive_msg.drive.speed = np.clip(drive_msg.drive.speed, -max_vel, max_vel)
    set_rc_remote(RemoteMode.drive)
    time_clb = 0.0

def set_rc_remote(mode =  RemoteMode.vel):
    """
    Recalculation velocity data to pulse and set to PWM servo and motor
    :return:
    """
    global vel_msg, pwm_msg, \
        intercept_remote, revers_val, \
        max_steering_angle, wheelbase, drive_msg, pwm_output_msg

    if mode == RemoteMode.pwm:
        pwm_output_msg = pwm_msg
        if(pwm_msg.ServoPWM > 0):
                pi.set_servo_pulsewidth(servo_pin, pwm_msg.ServoPWM)
        if(pwm_msg.MotorPWM > 0):
            pi.set_servo_pulsewidth(motor_pin, pwm_msg.MotorPWM)
    elif mode == RemoteMode.vel:
            # send servo
            # v = vel_msg.linear.x-vel_msg.linear.y
            # steering = convert_trans_rot_vel_to_steering_angle(v,vel_msg.angular.z, wheelbase)
        servo_val = valmap(math.degrees(vel_msg.angular.z), max_steering_angle*revers_val, max_steering_angle*-revers_val, 1000+offset, 2000+offset)
        pwm_output_msg.ServoPWM = servo_val
        try:
                pi.set_servo_pulsewidth(servo_pin, servo_val)
        except:
            print("error:", servo_val)
            # send motor
        if(intercept_remote and 0.0 <= vel_msg.linear.x < 0.1):
           print("return")
           return
        motor_val = valmap(vel_msg.linear.x, -2.4, 2.4, 1400, 1700, False)
        pi.set_servo_pulsewidth(motor_pin, motor_val)
        pwm_output_msg.MotorPWM = motor_val

    elif mode == RemoteMode.drive:
            # send servo
            servo_val = valmap(math.degrees(drive_msg.drive.steering_angle), max_steering_angle * revers_val, max_steering_angle * -revers_val,
                               1000 + offset, 2000 + offset)
            pi.set_servo_pulsewidth(servo_pin, servo_val)

            # send motor
            if (intercept_remote and 0.0 <= drive_msg.drive.speed < 0.1):
                print("return")
                return
            motor_val = valmap(drive_msg.drive.speed, -2.4, 2.4, 1400, 1700, False)
            pi.set_servo_pulsewidth(motor_pin, motor_val)
            pwm_output_msg.ServoPWM = servo_val
            pwm_output_msg.MotorPWM = motor_val
    else:
        print("error")
    pwm_pub.publish(pwm_output_msg)

def valmap(value, istart, istop, ostart, ostop, clip_flag = True):
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
    try:
    	val = ostart + (ostop - ostart) * ((value - istart) / (istop - istart))
    except:
        print("map error", value, istart, istop, ostart, ostop)
        val = 0.0
    if clip_flag:
        return np.clip(val, ostart, ostop)
    else:
        return val

if __name__ == "__main__":
    try:
        rospy.init_node("rc_control")
        rate = rospy.Rate(rate)

        # get args from ros params
        cmd_vel_topic = rospy.get_param('~cmd_vel', cmd_vel_topic)
        pwm_topic = rospy.get_param('~pwm_topic', pwm_topic)
        servo_pin = rospy.get_param('~servo_pin', servo_pin)
        pwm_output_topic = rospy.get_param('~pwm_output_topic', pwm_output_topic)
        middle_servo = rospy.get_param('~middle_servo', middle_servo)
        offset = rospy.get_param('~servo_offset', offset)
        motor_pin = rospy.get_param('~motor_pin', motor_pin)
        middle_motor = rospy.get_param('~middle_motor', middle_motor)
        max_vel = rospy.get_param('~max_vel', max_vel)
        revers_servo = rospy.get_param('~revers_servo', revers_servo)
        min_vel = rospy.get_param('~min_vel', min_vel)
        max_steering_angle = rospy.get_param('~max_steering_angle', max_steering_angle)
        wheelbase = rospy.get_param('~wheelbase', wheelbase)
        drive_topic = rospy.get_param('~drive_topic', drive_topic)

        if revers_servo:
            revers_val = -1.0
        else:
            revers_val = 1.0

        rospy.Subscriber(cmd_vel_topic, Twist, vel_clb)
        rospy.Subscriber(pwm_topic, CarPwmContol, vel_clb_pwm)
        rospy.Subscriber(drive_topic, AckermannDriveStamped, vel_clb_drive)
        pwm_pub = rospy.Publisher(pwm_output_topic, CarPwmContol, queue_size=10)

        print ("RC_control params: \n"
               "cmd_vel_topic: %s \n"
               "pwm_toppic: %s \n"
               "drive_topic: %s \n"
               "pwm_output_topic: %s \n"
               "max_vel: %f \n"
               "min_vel: %f \n"
               "max_steering_angle: %f \n"
               "wheelbase: %f \n"
               "servo_pin: %d \n"
               "middle_servo: %d \n"
               "servo_offset: %d \n"
               "motor_pin: %d \n"
               "middle_motor: %d \n"
               "revers servo: %f \n"
               "===================\n" % (cmd_vel_topic,
                                        pwm_topic,
                                        drive_topic,
                                        pwm_output_topic,
                                        max_vel,
                                        min_vel,
                                        max_steering_angle,
                                        wheelbase,
                                        servo_pin,
                                        middle_servo,
                                        offset,
                                        motor_pin,
                                        middle_motor,
                                        revers_servo))
        while not rospy.is_shutdown():
            try:
                time_clb += 0.2
                if(time_clb > 1.0):     # if data does not come to close pwm
                    vel_msg = Twist()
                    set_rc_remote(RemoteMode.vel)
            except:
                time_clb += 0.2
                if(time_clb > 1.0):     # if something is wrong to close pwm
                    vel_msg = Twist()
                    set_rc_remote(RemoteMode.vel)
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
