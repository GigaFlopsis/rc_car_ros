#! /usr/bin/env python
# coding: utf-8
"""
This pos controller for like-car robot
"""

import math
import numpy as np


import rospy
from geometry_msgs.msg import Twist, Pose, TwistStamped, PoseStamped
from geometry_msgs.msg import Twist
import tf

#value
velocity = float()
cmd_vel_msg = Twist()
current_pose = Pose()
current_course = float()
goal_pose = Pose()

max_vel = 2.0 # m/s
min_vel = 0.0 # m/s

#errors
error_course = float()
error_dist = float()

#topics
cmd_vel_topic = "cmd_vel" # output topic
vel_topic = "velocity"
goal_topic = "move_base_simple/goal"
#tf
base_link = "map"
child_link = "base_link"

# geometry methods
def vector_from_course(rot):
    """
    Get vector of car direction
    :param rot: course angle
    :return: return vector
    """
    rotate = np.array([[math.cos(rot), -math.sin(rot)],
    [math.sin(rot), math.cos(rot)]])

    pos = np.array([[1.0],[0.0]])
    vector = np.dot(rotate,pos)
    return vector

def angle_between_vec(p1, p2):
    ang1 = np.arctan2(*p1[::-1])
    ang2 = np.arctan2(*p2[::-1])
    res = (ang1 - ang2) % (2 * np.pi)
    if res > math.pi:
        res -= 2 * math.pi
    if res < -math.pi:
        res += 2 * math.pi
    return res

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

    pos = np.array([[b.position.x-a.position.x],
                    [b.position.y - a.position.y]])
    dist = np.linalg.norm(pos)
    return dist

def get_errors():
    """
    Get errors for controller
    """
    global current_pose, goal_pose, current_course, \
        error_course, error_dist

    #get error from course
    v1 = vector_from_course(current_course)
    v2 = [goal_pose.position.x, goal_pose.position.y]
    error_course = angle_between_vec(v1 ,v2)

    #get error from distance
    error_dist = get_distance_to(current_pose, goal_pose)

    print ("error_course: %f \n"
           "error_dist: %f " % (error_course, error_dist))

# Controller
def get_control():
    """
    This is main controller
    :return: cmd_vel
    """
    global velocity, cmd_vel_msg, error_dist, error_course

    #motor: cmd_vel_msg.linear.x
    #angle: cmd_vel_msg.angular.z

    # clip velicity between min < cmd_vel < max
    cmd_vel_msg.linear.x = np.clip(cmd_vel_msg.linear.x, min_vel, max_vel)
    return  cmd_vel_msg


# ROS callback
def vel_clb(data):
    """
    Get current velocity from rc car
    :type data: TwistStamped
    """
    global velocity
    velocity = math.sqrt(data.twist.linear.x ** 2 + data.twist.linear.y ** 2)

def goal_clb(data):
    """
    Get goal pose
    :type data: PoseStamped
    """
    global goal_pose
    goal_pose = data.pose
    print ("new goal pose: %s" % (str(goal_pose.position)))

if __name__ == "__main__":

    # init ros node
    rospy.init_node('rc_pos_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # Get ros args
    name_node = rospy.get_name()
    vel_topic = rospy.get_param(name_node + '/vel_topic ', vel_topic)
    cmd_vel_topic = rospy.get_param(name_node + '/cmd_vel', cmd_vel_topic)
    goal_topic = rospy.get_param(name_node + '/goal_topic', goal_topic)
    max_vel = rospy.get_param(name_node + '/max_vel', max_vel)
    min_vel = rospy.get_param(name_node + '/min_vel', min_vel)
    base_link = rospy.get_param(name_node + '/base_link', base_link)
    child_link = rospy.get_param(name_node + '/child_link', child_link)

    # start subscriber
    rospy.Subscriber(vel_topic, TwistStamped, vel_clb)
    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    vec_pub = rospy.Publisher(cmd_vel_topic, Twist,  queue_size=10)

    listener = tf.TransformListener()
    old_ros_time = rospy.get_time()

    try:
        while not rospy.is_shutdown():
            dt = rospy.get_time() - old_ros_time

            try:
                (trans, rot) = listener.lookupTransform(base_link, child_link, rospy.Time(0))
                current_pose.position = trans
                current_pose.orientation = rot
                # convert euler from quaternion
                (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
                current_course = yaw
            except:
                print("tf not found")
                continue

            old_ros_time = rospy.get_time()

            get_errors()

            cmd_vel_msg = get_control()

            vec_pub.publish(cmd_vel_msg) # publish msgs to the robot
            rate.sleep()
    except KeyboardInterrupt:   # if put ctr+c
        exit(0)