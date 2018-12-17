#! /usr/bin/env python
# coding: utf-8
"""
This is Ros node shows plot from velocity and pwm output of rc car
"""

import time
from threading import Thread
import math
import matplotlib.pyplot as plt  # creation plots
import argparse, sys

import rospy
from geometry_msgs.msg import TwistStamped
from rc_bringup.msg import CarPwmContol

pwm = CarPwmContol()
pwm.MotorPWM = 1550

odometry_vel = TwistStamped()
vel_norm = float()

cmd_vel_topic = "velocity" # output topic
pwm_topic = "pwm"

class PwmThread(Thread):
    """
    A threading example
    """

    def __init__(self, name):
        """Инициализация потока"""
        Thread.__init__(self)
        self.name = name

    def run(self):
        """Запуск потока"""
        global pwm
        while 1:
            try:
                k = int(input("set PWM:"))
                pwm.MotorPWM = k
                print("PWM: %d" % pwm.MotorPWM)
            except:
                sys.exit()


def vel_clb(data):
    """
    Get current velocity from rc car
    :type data: TwistStamped
    """
    global odometry_vel,vel_norm
    vel = data
    vel_norm = math.sqrt(data.twist.linear.x**2 + data.twist.linear.y**2)

if __name__ == "__main__":

    # init ros node
    rospy.init_node('velocity_test', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    pwm_pub = rospy.Publisher(pwm_topic, CarPwmContol, queue_size=10)
    rospy.Subscriber(cmd_vel_topic, TwistStamped, vel_clb)

    # init thread for set PWM
    my_thread = PwmThread("input PWM")
    my_thread.start()

    # ini list
    rc_norm_vel_list = list()
    pwm_list = list()
    time_list = list()

    old_ros_time = rospy.get_time()
    t = 0
    try:
        while not rospy.is_shutdown():
            parser = argparse.ArgumentParser(description=__doc__)
            parser.add_argument('--exit', action='store_true', help='Exit graph')
            options = parser.parse_args()

            pwm_pub.publish(pwm)

            dt = rospy.get_time() - old_ros_time
            old_ros_time = rospy.get_time()
            t += dt

            # draw plot
            pwm_list.append(pwm.MotorPWM)
            rc_norm_vel_list.append(vel_norm)
            time_list.append(t)

            # Рисуем графики
            plt.ion()
            ax1 = plt.subplot(211)
            ax1.plot(time_list, rc_norm_vel_list)
            ax1.set_title('vel RC car')
            ax1.legend()
            ax1.grid()

            ax2 = plt.subplot(212)
            ax2.plot(time_list, pwm_list)
            ax2.set_title('PWM rc car')
            ax2.legend()
            ax2.grid()

            plt.show()
            plt.pause(1.0 / 40.0)

            rate.sleep()

            if options.exit:
                my_thread.join()
                sys.exit()
    except KeyboardInterrupt:   # if put ctr+c
        print("stop")
        my_thread.join()
        exit(0)