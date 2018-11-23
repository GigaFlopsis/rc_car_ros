#! /usr/bin/env python
# coding: utf-8

# This is pos controller for like-car robot


import math
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
#from visualisation_msgs.msg import Marker

route= list()
goal_pose = PoseStamped()
current_pose = PoseStamped()
local_finish_flag=False
first_waypoint_in_route = PoseStamped()
dist=0.0
goal_tolerance=0.2
#Markers=Marker()

#Topics
goal_topic = "route"
pose_topic = "pose"

def current_pose_clb(data):
    #Get current pose from topic
    global current_pose, current_course

    current_pose = data.pose
    rot = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    # # convert euler from quaternion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
    current_course = yaw

def goal_clb(data):
    global route, local_finish_flag, goal_pose, first_waypoint_in_route , last_waypoint_in_route, goal_tolerance
    goal_pose=data.pose
    route.append(goal_pose)
   # print("Leinght",len(route))
    first_waypoint_in_route = route[0]
    if abs(get_distance_to(first_waypoint_in_route,current_pose))<=goal_tolerance:
        del route[0]
    if len(route)==0:
        local_finish_flag=True
        route.clear()


#get distance to goal
def get_distance_to(a,b):
    """
    get distance to goal point
    :type a: Pose
    :type b: Pose
    :type dist: float
    :param a: current pose
    :param b: goal pose
    :param dist: distance
    :return: dist
    """
    pos = np.array([[b.position.x - a.position.x],
                    [b.position.y - a.position.y]])
    dist = np.linalg.norm(pos)
    return dist


def route_function():
    global goal_pose, goal_topic, route , first_waypoint_in_route
    pub=rospy.Publisher(goal_topic, Pose, queue_size=10)
    rospy.init_node('rc_route_generator', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # start subscriber
    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        pub.publish(first_waypoint_in_route)

    rate.sleep()
    

if __name__ == "__main__":
    try:
    rospy.init_node('rc_route_generator', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # start subscriber
    pub=rospy.Publisher(goal_topic, Pose, queue_size=10)
    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
    listener = tf.TransformListener()


        old_ros_time = rospy.get_time()
    currentTime = 0.0
    rate.sleep()

    try:
        while not rospy.is_shutdown():
            dt = rospy.get_time() - old_ros_time
            currentTime += dt


            if(not init_flag):
                if currentTime > 1.0:
                    print("pose controller: not init")
                    currentTime =  0.0
                continue

            old_ros_time = rospy.get_time()

            route_function()
           
            if finish_flag:
                if currentTime > 1.0:
                    print("pose controller: finish_flag True")
                    currentTime = 0.0
                cmd_vel_msg.linear.x = 0.0
                init_flag = False

            pub.publish(first_waypoint_in_route)
            rate.sleep()

    except KeyboardInterrupt:   # if put ctr+c
        exit(0)

