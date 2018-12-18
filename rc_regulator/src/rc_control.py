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
import serial
from enum import Enum

import rospy
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from rc_car_msgs.msg import CarParams, CarPwmContol
from std_srvs.srv import SetBool
from PID import PID

from dynamic_reconfigure.server import Server
from rc_bringup.cfg import RcVelConcfg_srvtrollerConfig
from std_msgs.msg import Float64

class RemoteMode(Enum):
    odometry_vel = 0
    pwm = 1
    drive = 2

# init params
servo_pin = 22 # inut pin of servo
motor_pin = 4 # inut pin of motor
motor_run = True
use_odometry_vel = False

middle_servo = 1550
middle_motor = 1550 # for rc550
#middle_motor = 1500 #for rc540
offset = 47.0 # offset of servo
revers_servo = True # revers of servo direction
revers_val = 1.0
max_vel = 2.5 # max speed of the car
min_vel = -2.5 # min speed of the car
max_angle = 25.0 # in degrees
wheelbase = 0.28 # in meters
prev_vel = 0.0
odometry_vel=float()

current_course = float()

# init PID
motor_pid = PID()
motor_pid.setWindup(500)

kP = 1.0
kI = 0.0
kD = 0.2

#topics
cmd_vel_topic = "/cmd_vel"
vel_topic = "/mavros/local_position/velocity"
goal_topic = "/goal"
pose_topic = "/mavros/local_position/pose"
encoder_vel_topic = "/encoder_vel"

# init topics
cmd_vel_topic = "/cmd_vel"       # remote via velocity
pwm_topic = "/pwm"               # direct remote PWM
drive_topic ="/ackermann_cmd"   # remote like-car
pwm_output_topic = "/pwm_output"   # remote like-car
vel_topic = "/mavros/local_position/velocity"
param_topic = "/params"
setmode_srv_topic = "/car/set_mode"

# PWM init
pi = pigpio.pi()
pi.set_servo_pulsewidth(servo_pin, middle_servo) # middle servo angle
pi.set_servo_pulsewidth(motor_pin, middle_motor) # zero speed for motor (different depending on ESC)

# init value
current_mode = RemoteMode.vel
current_velocity = TwistStamped()   # vector of velocity
norm_velocity = float()             # in m/s

goal_vel_msg = Twist()
pwm_msg = CarPwmContol()
drive_msg = AckermannDriveStamped()
pwm_output_msg = CarPwmContol()

hz = 50
time_clb = 0.0
vel1=float()
vel2=float()
odometry_vel=float()

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
    vel_msg = data
    vel_msg.linear.x = np.clip(vel_msg.linear.x, min_vel, max_vel)
    current_mode = RemoteMode.vel
    time_clb = 0.0
#    print("vel_msg.linear.x", vel_msg.linear.x)

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

#def drive_vel_clb(data):
#    """
#       Get drive value from topic
#       :param data: velocity and  steering value
#       :type data: AckermannDriveStamped
#       """
#    global drive_msg,time_clb, max_vel, current_mode
#    drive_msg = data
#    drive_msg.drive.speed = np.clip(drive_msg.drive.speed, -max_vel, max_vel)
#    current_mode = RemoteMode.drive
#    time_clb = 0.0

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

def encoder_vel_clb(data):
    global odometry_vel
    vel=data.data

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

def SetModeSrv_clb(req):
    """
    Servoce for set mode of regulator
    :param req:
    :return:
    """
    global motor_run, intercept_remote
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

    use_imu_vel = bool(config["use_imu_vel"])
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
        max_angle, wheelbase, drive_msg, pwm_output_msg, prev_vel, use_odometry_vel, motor_pid,odometry_vel

    motor_val = 0.0
    
    if mode == RemoteMode.vel:
        servo_val = valmap(vel_msg.angular.z, max_angle * revers_val, max_angle * -revers_val, 1000 + offset, 2000 + offset)

        pwm_output_msg.ServoPWM = servo_val
        try:
            pi.set_servo_pulsewidth(servo_pin, servo_val)
        except:
            print("error:", servo_val)

        if use_imu_vel:
        # PID controlled
        # motor_val = valmap(vel_msg.linear.x, -2.4, 2.4, 1200, 1600, False)
            setPIDk() #set PID coefficients
            print("check vel",vel)
            error_vel = vel - vel_msg.linear.x
            motor_pid.update(error_vel)
            motor_val = motor_pid.output + middle_motor
        #print("incoder vel", vel)
        #else:
        # use relative velocity
        #    if vel_msg.linear.x >= 0.0:
        #        motor_val = valmap(vel_msg.linear.x, 0.0 , 1.0, middle_motor, 1700, False)
        #    if vel_msg.linear.x < 0.0:
        #        motor_val = valmap(vel_msg.linear.x, -1.0, 0.0, 1400, middle_motor, False) #1400
#        print("send vel", motor_val)

    # Send to pwm motor
        pi.set_servo_pulsewidth(motor_pin, motor_val)
        motor_val = np.clip(motor_val, 1000, 2000)
        print("motor_val",motor_val)
    if motor_val<=1569:
        motor_val=1570
        pwm_output_msg.MotorPWM = motor_val
        prev_vel = vel_msg.linear.x #read prev velocity value

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
     #   rospy.Subscriber(drive_topic, AckermannDriveStamped, drive_vel_clb)
        rospy.Subscriber(vel_topic, TwistStamped, velocity_clb)
        rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
        rospy.Subscriber(encoder_vel_topic,Float64, encoder_vel_clb)

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
                    set_rc_remote(current_mode)
                    #print('1')     # set pwm mode
                    
                    
                   # print("value",value)

                else:           # not cld remote data break pwm
                    pi.set_servo_pulsewidth(servo_pin, 0)
                    pi.set_servo_pulsewidth(motor_pin, 0)
            except:
                pi.set_servo_pulsewidth(servo_pin, 0)
                pi.set_servo_pulsewidth(motor_pin, middle_motor)
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

 
