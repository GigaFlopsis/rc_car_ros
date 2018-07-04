#! /usr/bin/env python
# coding: utf-8
"""
This pos controller for like-car robot
"""

import math
import numpy as np
from PID import PID
import time
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, TwistStamped, PoseStamped
from geometry_msgs.msg import Twist
from dynamic_reconfigure.server import Server
from rc_bringup.cfg import PoseControllerConfig

#value
velocity = float()
cmd_vel_msg = Twist()
current_pose = Pose()
current_course = float()
goal_pose = Pose()
init_flag = False
goal_tolerance = 0.4

finish_flag = True


#topics
goal_topic = "move_base_simple/goal"
#tf
base_link = "map"
child_link = "goal"
current_pose = PoseStamped()

if __name__ == "__main__":

    # init ros node
    rospy.init_node('rc_pos_controller', anonymous=True)
    rate = rospy.Rate(20)  # 10hz

    # Get ros args
    name_node = rospy.get_name()
    goal_topic = rospy.get_param(name_node + 'goal_topic', goal_topic)
    base_link = rospy.get_param(name_node + 'base_link', base_link)
    child_link = rospy.get_param(name_node + 'child_link', child_link)




    # start subscriber
    rospy.Subscriber(vel_topic, TwistStamped, vel_clb)
    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    goal_pub = rospy.Publisher(goal_topic, PoseStamped,  queue_size=10)

    listener = tf.TransformListener()

    try:
        while not rospy.is_shutdown():

            try:
                (trans, rot) = listener.lookupTransform(base_link, child_link, rospy.Time(0))
                current_pose.pose.position.x = trans[0]
                current_pose.pose.position.y = trans[1]
                current_pose.pose.position.z = trans[2]
                current_pose.pose.orientation = rot
                # convert from quaternion
                goal_pub.publish(current_pose)
            except:
                print("tf not found")
                continue
            rate.sleep()
    except KeyboardInterrupt:   # if put ctr+c
        exit(0)


