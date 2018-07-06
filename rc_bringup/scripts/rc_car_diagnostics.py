#!/usr/bin/env python
# coding: utf-8

import rospy
from rospy import Header
from mavros_msgs.msg import State, HomePosition
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from rc_car_msgs.msg import Diagnostics, CarParams



diag_msg = Diagnostics()


# topic name
diag_topic = "diagnostics"
gps_topic = "mavros/global_position/global"
battery_topic = "/mavros/battery"
state_topic = "/mavros/state"
home_gps_topic = "/mavros/home_position/home"
origin_topic = "/geo/set_origin"
params_topic = "/params"

# timer init
state_timer = 0.0
battery_timer = 0.0
gps_timer = 0.0
home_gps_timer = 0.0

# init delay
state_delay = 2.0
battery_delay = 2.0
gps_delay = 0.5
home_gps_delay = 3.0


# callback data
def gps_home_clb(data):
    global home_gps_timer, diag_msg
    home_gps_timer = 0.0
    diag_msg.init_home = True

def gps_clb(data):
    global gps_timer, diag_msg
    gps_timer = 0.0
    diag_msg.gps_send = True

def battery_clb(data):
    """
    :type diag_msg: Diagnostics
    :type data: BatteryState

    """
    global battery_timer, diag_msg
    battery_timer = 0.0
    diag_msg.battery = data.percentage

def state_clb(data):
    """
    :type diag_msg: Diagnostics
    :type data: State
    :return:
    """
    global state_timer, diag_msg
    state_timer = 0.0

    diag_msg.armed = data.armed
    diag_msg.mode = data.mode

def origin_clb(data):
    global diag_msg
    diag_msg.init_origin = True

def controller_params_clb(data):
    """
    Get params data from rc controller
    :param data:
    :return:
    """
    global diag_msg
    diag_msg.params = data

if __name__ == '__main__':
    rospy.init_node('drone_diagnosrics_node', anonymous=True)
    rate = rospy.Rate(20)
    rospy.Subscriber(gps_topic, NavSatFix, gps_clb)
    rospy.Subscriber(battery_topic, BatteryState, battery_clb)
    rospy.Subscriber(state_topic, State, state_clb)
    rospy.Subscriber(home_gps_topic, HomePosition, gps_home_clb)
    rospy.Subscriber(origin_topic, NavSatFix, origin_clb)
    rospy.Subscriber(params_topic, CarParams, )

    diag_pub = rospy.Publisher(diag_topic,Diagnostics, queue_size=10)

    old_time = rospy.get_time()
    try:
        while not rospy.is_shutdown():
            dt = rospy.get_time() - old_time
            old_time = rospy.get_time()

            home_gps_timer += dt
            state_timer += dt
            gps_timer += dt
            battery_timer += dt

            if home_gps_timer > home_gps_delay:
                diag_msg.init_home = False
            if gps_timer > gps_delay:
                diag_msg.gps_send = False

            diag_msg.header.stamp = rospy.Time.now()
            diag_msg.header.frame_id = "base_link"
            diag_pub.publish(diag_msg)
            rate.sleep()

    except:
         print("exit")
         raise
