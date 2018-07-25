#! /usr/bin/env python
# coding: utf-8
"""
This is Ros node for pwm control rc car
"""
from enum import Enum
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TwistStamped
from rc_car_msgs.msg import CarParams, CarPwmContol
import math
import numpy as np
import rospy
import time

from geometry_msgs.msg import TwistStamped
import tf
import matplotlib.pyplot as plt  # creation plots
import copy

# ---------------------------------------------------------------------------------------------------------------------
# ---------- Глобальные переменные


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Функции подписки на топики

norm_velocity = 0.0
norm_acceleration = 0.0
current_pwm = CarPwmContol()

srez = -100
# ---------------------------------------------------------------------------------------------------------------------
# ---------- Вспомогательные функции
def velocity_clb(data):
    """
    Get current velocity from FCU
    :param data: velocity from NED
    """
    global norm_velocity
    norm_velocity = np.linalg.norm(np.array([data.twist.linear.x,
                                             data.twist.linear.y]))

def imu_clb(data):
    """
    Get current velocity from FCU
    :param data: velocity from NED
    """
    global norm_acceleration
    norm_acceleration = np.linalg.norm(np.array([data.linear_acceleration.x,
                                                 data.linear_acceleration.y]))

def pmw_motor_clb(data):
    """
    Get current velocity from FCU
    :param data: velocity from NED
    """
    global current_pwm
    current_pwm = data


# ---------------------------------------------------------------------------------------------------------------------
# ---------- Главная функция
def main():
    """
    Основной цикл узла ROS.

    @return: код завершения программы
    """
    # Инициализируем узел ROS
    rospy.init_node("rc_reg_plotter")

    # Подписываемся на топики
    rospy.Subscriber("/pwm_output", CarPwmContol, pmw_motor_clb)
    rospy.Subscriber("/mavros/imu/data", Imu, imu_clb)
    rospy.Subscriber("mavros/local_position/velocity", TwistStamped, velocity_clb)

    # Инициализация "слушателя" навигационных преобразований
    tf_listener = tf.TransformListener()

    # Инициализируем спискиpwm_output для построения графиков
    velocity_list = list()
    accel_list = list()
    pwm_list = list()
    time_plot = list()

    old_ros_time = time.time()
    # Основной цикл
    rate = rospy.Rate(10)
    t_ = 0.0
    while not rospy.is_shutdown():
        # Сичтываем навигационные данные


        # Рассчитываем время
        dt = time.time() - old_ros_time
        old_ros_time = time.time()

        t_ += dt
        velocity_list.append(norm_velocity)
        accel_list.append(norm_acceleration)
        pwm_list.append(current_pwm.MotorPWM)
        time_plot.append(t_)
	if srez > 0:
		velocity_list = velocity_list[srez::]
		accel_list = accel_list[srez::]
		pwm_list = pwm_list[srez::]
		time_plot = time_plot[srez::]

        # print ('t %s' % t_)

        # Рисуем графики
        ax1 = plt.subplot2grid((2,1), (0,0))
        ax1.plot(time_plot, velocity_list, label='velocity')
        ax1.plot(time_plot, accel_list, label='accel')
        ax1.set_ylabel('x, m/s')
        ax1.set_xlabel('t, s')
        ax1.legend()
        ax1.grid()

        ax2 = plt.subplot2grid((2,1), (1,0))
        ax2.plot(time_plot, pwm_list, label='pwm')
        ax2.set_title('PWM')
        ax2.set_ylabel('y, ms')
        ax2.set_xlabel('t, s')
        ax2.legend()
        ax2.grid()

        # ax3 = plt.subplot2grid((3, 3), (0, 2))
        # ax3.plot(time_plot, drone_z, label='drone z')
        # ax3.plot(time_plot, goal_z, label='goal z')
        # ax3.set_title('Coords: drone Z and goal Z')
        # ax3.set_ylabel('z, m')
        # ax3.set_xlabel('t, s')
        # ax3.legend()
        # ax3.grid()

        plt.pause(1.0 / 10.0)

        # rate.sleep()
    return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
