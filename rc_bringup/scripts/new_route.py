#! /usr/bin/env python
# coding: utf-8

# This is pos controller for like-car robot


import math
import os
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
init_flag = False
goal_pose_prev=PoseStamped()
#Markers=Marker()

#Topics
goal_topic = "goal"
pose_topic = "pose"
point=PoseStamped()

def current_pose_clb(data):
    #Get current pose from topic
    global current_pose, current_course

    current_pose = data
    rot = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    # # convert euler from quaternion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
    current_course = yaw

def goal_clb(data):
    global route, local_finish_flag, goal_pose, first_waypoint_in_route , last_waypoint_in_route, goal_tolerance,init_flag , finish_flag,goal_pose_prev
    goal_pose=data
    init_flag = True
    finish_flag = False
    #print("Leinght",len(route))
    #first_waypoint_in_route = route[0]

    #if goal_pose!=goal_pose_prev:
    route.append(goal_pose)
    print("first waypoint in row", route[0])

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
    pos = np.array([[b.pose.position.x - a.pose.position.x],
                    [b.pose.position.y - a.pose.position.y]])
    dist = np.linalg.norm(pos)
    return dist

def route_clb(data):
    global point
    if goal_pose!=0:
        rospy.sleep(0)
    point=data
    init_flag = True

def calc_route():
    global point, route
    route.append(point)
    #if get_distance_to(current_pose,route[0])==0.5:
    print("distance cur_p and route [0]",get_distance_to(current_pose,route[0]))
    if get_distance_to(current_pose,route[0]) < 0.2:
        del route[0]
    if len(route)>1:
        if get_distance_to(route[-1],route[-2]) < 0.3:
            del route[-2]
    if len(route)>=1:
        first_waypoint_in_route=route[0]



if __name__ == "__main__":
    rospy.init_node('rc_route_generator', anonymous=True)
    rate=rospy.Rate(10)
    pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
    #rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
    rospy.Subscriber('/route',PoseStamped,route_clb)
    listener = tf.TransformListener()
    old_ros_time = rospy.get_time()
    currentTime = 0.0
    rate.sleep()
    try:
        while not rospy.is_shutdown():
            dt = rospy.get_time() - old_ros_time
            currentTime += dt

            #if(not init_flag):
             #   if currentTime > 1.0:
             #       print('init_flag',init_flag)
             #       print("route: not init")
             #       currentTime =  0.0
             #   continue
            old_ros_time = rospy.get_time()
            calc_route()
            print("Length",len(route))
            if len(route)>=1:
                pub.publish(route[0])
                print("FIRST =",route[0])
            rate.sleep()


    except KeyboardInterrupt:   # if put ctr+c
        exit(0)
