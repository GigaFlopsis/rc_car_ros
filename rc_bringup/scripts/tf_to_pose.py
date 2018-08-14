#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Get linear speed from TF
"""

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist

import copy

pose_topic = "pose"
base_link = "origin"
child_link = "marker"

prev_pose = list()
current_pos = PoseStamped()


listener = tf.TransformListener()

if __name__ == '__main__':

    rospy.init_node('tf_to_pose_node', anonymous=True)

    # Get ros args
    name_node = rospy.get_name()
    pose_topic = rospy.get_param('~pose_topic', pose_topic)
    base_link = rospy.get_param('~base_link', base_link)
    child_link = rospy.get_param('~child_link', child_link)

    # Publisher to topic
    pose_pub = rospy.Publisher(pose_topic, PoseStamped, queue_size=1)

    old_time = rospy.get_time()
    rate = rospy.Rate(10)   # 10 hz

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform(base_link, child_link, rospy.Time(0))
            current_pos.header = rospy.Header()
            current_pos.header.stamp = rospy.Time.now()
            current_pos.header.frame_id = "map"
            current_pos.pose.position.x = trans[0]
            current_pos.pose.position.y = trans[1]
            current_pos.pose.position.z = trans[2]
            current_pos.pose.orientation.x = rot[0]
            current_pos.pose.orientation.y = rot[1]
            current_pos.pose.orientation.z = rot[2]
            current_pos.pose.orientation.w = rot[3]
            pose_pub.publish(current_pos)
            print ("pose:", current_pos.pose.position)
        except:
            print ("false")

        rate.sleep()
