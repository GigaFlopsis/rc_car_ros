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
import tf
from enum import Enum

import rospy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from rc_car_msgs.msg import CarParams, CarPwmContol
from std_srvs.srv import SetBool
from PID import PID
from std_msgs.msg import Float64

from dynamic_reconfigure.server import Server
from rc_bringup.cfg import RcVelControllerConfig


class RemoteMode(Enum):
    odometry_vel = 0
    pwm = 1
    drive = 2

# init params
servo_pin = 4           # inut pin of servo
motor_pin = 17          # inut pin of motor
motor_run = True        # enable/disable motor
use_odometry_vel = False    # use odometry velocity
disable_stop = True         # clip pwm output true = midle_motor -> 2000; false = 1000 -> 2000

middle_servo = 1550
middle_motor = 1550 # for rc550
offset = 47.0 # offset of servo
revers_servo = False # revers of servo direction
revers_val = 1.0
max_vel = 2.5 # max speed of the car
min_vel = -2.5 # min speed of the car
max_angle = 25.0 # in degrees
wheelbase = 0.28 # in meters

# PID params
kP = 1.0
kI = 0.0
kD = 0.2

hz = 50

prev_vel = 0.0
current_course = float()

# init PID
motor_pid = PID()
motor_pid.setWindup(500)

#topics
cmd_vel_topic = "/cmd_vel"
vel_topic = "/mavros/local_position/velocity"
goal_topic = "/goal"
pose_topic = "/mavros/local_position/pose"


# init topics
cmd_vel_topic = "/cmd_vel"       # remote via velocity
pwm_topic = "/pwm"               # direct remote PWM
drive_topic ="/ackermann_cmd"   # remote like-car
pwm_output_topic = "/pwm_output"   # remote like-car
vel_topic = "/mavros/local_position/velocity"
param_topic = "/params"
setmode_srv_topic = "/car/set_mode"
encoder_topic = "/encoder_vel"

# PWM init
pi = pigpio.pi()
pi.set_servo_pulsewidth(servo_pin, middle_servo) # middle servo angle
pi.set_servo_pulsewidth(motor_pin, middle_motor) # zero speed for motor (different depending on ESC)

# init value
current_mode = RemoteMode.vel
current_velocity = TwistStamped()   # vector of velocity
norm_velocity = float()             # in m/s
odometry_vel = float()              # velosity from odometry

goal_vel_msg = Twist()
pwm_msg = CarPwmContol()
drive_msg = AckermannDriveStamped()
pwm_output_msg = CarPwmContol()

time_clb = 0.0

def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
  global max_angle
  if omega == 0.0:
    return 0
  if v == 0.0:
    return math.degrees(omega)

  radius = v / omega
  steering_angle = math.degrees(math.atan(wheelbase / radius))
  return np.clip(steering_angle, -max_angle, max_angle)


## Callbeck from ROS

def cmd_vel_clb(data):
    """
    Get velocity value from topic
    :param data: velocity value
    :type data: Twist

    """
    global goal_vel_msg, time_clb, max_vel, min_vel, current_mode
    goal_vel_msg = data
    goal_vel_msg.linear.x = np.clip(goal_vel_msg.linear.x, min_vel, max_vel)
    current_mode = RemoteMode.vel
    time_clb = 0.0
#    print("goal_vel_msg.linear.x", goal_vel_msg.linear.x)

def pwm_clb(data):
    """car_break_topic
    Get PWM value from topic
    :param data: velocity value
    :type data: RcCarControl

    """
    global pwm_msg, time_clb, current_mode
    pwm_msg = data
    current_mode = RemoteMode.pwm
    time_clb = 0.0

def drive_vel_clb(data):
    """
       Get drive value from topic
       :param data: velocity and  steering value
       :type data: AckermannDriveStamped
       """
    global drive_msg,time_clb, max_vel, current_mode
    drive_msg = data
    drive_msg.drive.speed = np.clip(drive_msg.drive.speed, -max_vel, max_vel)
    current_mode = RemoteMode.drive
    time_clb = 0.0

def current_pose_clb(data):
    """
    Get current pose from topic
    :param data:
    :return:
    """
    global current_course
    rot = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    # convert euler from quaternion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
    current_course = yaw

def velocity_clb(data):
    """
    Get current velocity from FCU
    :param data: velocity from NED
    """
    global time_clb, current_mode, current_velocity, norm_velocity, current_course

    rot=current_course
    current_velocity = data
    rotate=np.array([[math.cos(rot),-math.sin(rot)],
                     [math.sin(rot),math.cos(rot)]])
    velocity=np.array([[data.twist.linear.x],
                       [-data.twist.linear.y]])
    vector=np.dot(rotate,velocity)
    norm_velocity =vector[0]
#    print("vector:",vector[0],vector[1])
    current_mode = RemoteMode.vel
    time_clb = 0.0

def encoder_clb(data):
    global odometry_vel
    odometry_vel=data

def SetModeSrv_clb(req):
    """
    Servoce for set mode of regulator
    :param req:
    :return:
    """
    global motor_run
    motor_run = req.data

def cfg_callback(config, level):
    """
    Get params from dynamic reconfigure
    :param config:
    :param level:
    :return:
    """

    global max_vel, min_vel, max_angle, kP, kI, kD, offset, use_odometry_vel, motor_run

    print("config")
    max_vel = float(config["max_vel"])
    min_vel = float(config["min_vel"])
    max_angle = math.radians(float(config["max_angle"]))
    offset = float(config["servo_offset"])

    use_odometry_vel = bool(config["use_imu_vel"])
    motor_run = bool(config["motor_run"])

    kP = float(config["kP"])
    kI = float(config["kI"])
    kD = float(config["kD"])

    return config

## Other function

def setPIDk():
    """
    update PID coefficients
    :return:
    """
    global motor_pid, kP, kI, kD

    motor_pid.setKp(kP)
    motor_pid.setKi(kI)
    motor_pid.setKd(kD)

def set_rc_remote(mode):
    """
    Recalculation velocity data to pulse and set to PWM servo and motor
    :return:
    """
    global goal_vel_msg, pwm_msg, \
        intercept_remote, revers_val, \
        max_angle, wheelbase, drive_msg, pwm_output_msg, prev_vel, use_odometry_vel, motor_pid,odometry_vel, disable_stop

    motor_val = 0.0

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
        servo_val = valmap(goal_vel_msg.angular.z, max_angle * revers_val, max_angle * -revers_val, 1000 + offset, 2000 + offset)

        pwm_output_msg.ServoPWM = servo_val
        try:
                pi.set_servo_pulsewidth(servo_pin, servo_val)
        except:
            print("error:", servo_val)
            # send motor

        # # send motor data
        # ## check input velocity data
        # if (0.0 <= vel_msg.linear.x < 0.1):  # if target velocity a small, breake speed
        #     pi.set_servo_pulsewidth(motor_pin, middle_motor)
        #     pwm_output_msg.MotorPWM = middle_motor
        #     pass

        ## stop motor correction
       # if prev_vel > 0 and vel_msg.linear.x <= 0.0: #for forward moving brake
       #     print("stop motor")
       #     motor_val = 1300
       #     pi.set_servo_pulsewidth(motor_pin, motor_val)
       #     pwm_output_msg.MotorPWM = motor_val
       #     print("val 1:", motor_val)
       #     time.sleep(0.5) #first signal need to repay previous value on engine
       #     pi.set_servo_pulsewidth(motor_pin, middle_motor)
       #     pwm_output_msg.MotorPWM = middle_motor
       #     print("val 2:", motor_val)
       #     time.sleep(0.5) #second to stop the car

        if use_odometry_vel:
            # PID controlled
            # motor_val = valmap(vel_msg.linear.x, -2.4, 2.4, 1200, 1600, False)
            setPIDk() #set PID coefficients
            error_vel = odometry_vel - goal_vel_msg.linear.x
            motor_pid.update(error_vel)
            motor_val = motor_pid.output + middle_motor
        else:
            # use relative velocity
            if goal_vel_msg.linear.x >= 0.0:
                motor_val = valmap(goal_vel_msg.linear.x, 0.0, 1.0, middle_motor, 1700, False)
            if goal_vel_msg.linear.x < 0.0:
                motor_val = valmap(goal_vel_msg.linear.x, -1.0, 0.0, 1400, middle_motor, False) #1400

        # Send to pwm motor
        if disable_stop:
            motor_val = np.clip(motor_val, middle_motor, 2000)
        else:
            motor_val = np.clip(motor_val, 1000, 2000)

        pi.set_servo_pulsewidth(motor_pin, motor_val)
        pwm_output_msg.MotorPWM = motor_val
        prev_vel = goal_vel_msg.linear.x         #read prev velocity value

    elif mode == RemoteMode.drive:
        # send servo
        v = drive_msg.drive.speed
        steering = convert_trans_rot_vel_to_steering_angle(v, drive_msg.drive.steering_angle, wheelbase)
        servo_val = valmap(steering, max_angle * revers_val, max_angle * -revers_val,
                           1000 + offset, 2000 + offset)
        pi.set_servo_pulsewidth(servo_pin, servo_val)
        pwm_output_msg.ServoPWM = servo_val

        # send motor data
        # ## check input velocity data
        # if (-0.1 <= drive_msg.drive.speed < 0.1):  # if target velocity a small, breake speed
        #     pi.set_servo_pulsewidth(motor_pin, middle_motor)
        #     pwm_output_msg.MotorPWM = middle_motor
        #     print("break motor")
        #     pass

        ## stop motor correction
        if prev_vel > 0 and drive_msg.drive.speed <= 0.0: #for forward moving brake
            motor_val = 1300
            pi.set_servo_pulsewidth(motor_pin, motor_val)
            pwm_output_msg.MotorPWM = motor_val
            time.sleep(0.5) #first signal need to repay previous value on engine
            pi.set_servo_pulsewidth(motor_pin, middle_motor)
            pwm_output_msg.MotorPWM = middle_motor
            time.sleep(0.5) #second to stop the car

        if use_odometry_vel:
            # PID controlled
            # motor_val = valmap(vel_msg.linear.x, -2.4, 2.4, 1200, 1600, False)
            setPIDk() #set PID coefficients
            error_vel = norm_velocity + drive_msg.drive.speed
            motor_pid.update(error_vel)
            motor_val = motor_pid.output + middle_motor
        else:
            # use relative velocity
            if drive_msg.drive.speed >= 0.0:
                motor_val = valmap(drive_msg.drive.speed, 0.0, 6.0, middle_motor, 1700, False)
            if drive_msg.drive.speed < 0.0:
                motor_val = valmap(drive_msg.drive.speed, -2.0, 0.0,1300, middle_motor, False)

        # Send to pwm motor
        motor_val = np.clip(motor_val, 1000, 2000)
        pi.set_servo_pulsewidth(motor_pin, motor_val)
        pwm_output_msg.MotorPWM = motor_val
        prev_vel = drive_msg.drive.speed #read prev velocity value

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

def get_car_params():
    """
    Get car params data
    :return: CarParams
    """
    data = CarParams()
    data.motor_run = motor_run
    data.maxSteeringAngle = max_angle
    data.maxVel = max_vel
    data.wheelbase = wheelbase
    return data


if __name__ == "__main__":
    try:
        rospy.init_node("rc_control")
        rate = rospy.Rate(hz)

        # init dynamic reconfigure server
        cfg_srv = Server(RcVelControllerConfig, cfg_callback)


        # get args from ros params
        ## topics
        cmd_vel_topic = rospy.get_param('~cmd_vel', cmd_vel_topic)
        pwm_topic = rospy.get_param('~pwm_topic', pwm_topic)
        pwm_output_topic = rospy.get_param('~pwm_output_topic', pwm_output_topic)
        drive_topic = rospy.get_param('~drive_topic', drive_topic)
        param_topic = rospy.get_param('~param_topic', param_topic)
        vel_topic = rospy.get_param('~vel_topic', vel_topic)

        ## GPIO
        servo_pin = rospy.get_param('~servo_pin', servo_pin)
        motor_pin = rospy.get_param('~motor_pin', motor_pin)
        middle_servo = rospy.get_param('~middle_servo', middle_servo)
        middle_motor = rospy.get_param('~middle_motor', middle_motor)
        revers_servo = rospy.get_param('~revers_servo', revers_servo)
        offset = rospy.get_param('~servo_offset', offset)

        ## rover params

        wheelbase = rospy.get_param('~wheelbase', wheelbase)
        if rospy.has_param('~max_vel'):
            max_vel = rospy.get_param('~max_vel', max_vel)
            cfg_srv.update_configuration({"max_vel": max_vel})
        if rospy.has_param('~min_vel'):
            min_vel = rospy.get_param('~min_vel', min_vel)
            cfg_srv.update_configuration({"min_vel": min_vel})
        if rospy.has_param('~max_angle'):
            max_angle = rospy.get_param('~max_angle', max_angle)
            cfg_srv.update_configuration({"max_angle": max_angle})

        ## PID params
        if rospy.has_param('~use_imu_vel'):
            use_odometry_vel = rospy.get_param('~use_imu_vel', use_odometry_vel)
            cfg_srv.update_configuration({"use_imu_vel": use_odometry_vel})
        if rospy.has_param('~kP'):
            kP = rospy.get_param('~kP', kP)
            cfg_srv.update_configuration({"kP": kP})
        if rospy.has_param('~kI'):
            kI = rospy.get_param('~kI', kI)
            cfg_srv.update_configuration({"kI": kI})
        if rospy.has_param('~kD'):
            kD = rospy.get_param('~kD', kD)
            cfg_srv.update_configuration({"kD": kD})

        if revers_servo:
            revers_val = -1.0
        else:
            revers_val = 1.0

        # Subscribe and Publisher to topics
        rospy.Subscriber(cmd_vel_topic, Twist, cmd_vel_clb)
        rospy.Subscriber(pwm_topic, CarPwmContol, pwm_clb)
        rospy.Subscriber(drive_topic, AckermannDriveStamped, drive_vel_clb)

        rospy.Subscriber(vel_topic, TwistStamped, velocity_clb)
        rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
        rospy.Subscriber(encoder_topic, Float64, encoder_clb)
        pwm_pub = rospy.Publisher(pwm_output_topic, CarPwmContol, queue_size=10)
        param_pub = rospy.Publisher(param_topic, CarParams, queue_size=10)

        s = rospy.Service(setmode_srv_topic, SetBool, SetModeSrv_clb)


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
                                          max_angle,
                                          wheelbase,
                                          servo_pin,
                                          middle_servo,
                                          offset,
                                          motor_pin,
                                          middle_motor,
                                          revers_servo))
        while not rospy.is_shutdown():
            try:
                time_clb += 1.0 / hz

                if time_clb < 1.0 and motor_run:
                    set_rc_remote(current_mode)     # set pwm mode

                else:           # not cld remote data break pwm
                    pi.set_servo_pulsewidth(servo_pin, 0)
                    pi.set_servo_pulsewidth(motor_pin, 0)
            except:
                pi.set_servo_pulsewidth(servo_pin, 0)
                pi.set_servo_pulsewidth(motor_pin, 0)
                print("rc control: error")

            param_pub.publish(get_car_params())     #publish car params from topic
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
