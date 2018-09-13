#! /usr/bin/env python
# coding: utf-8

# This is pos controller for like-car robot


import math
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan

#value
velocity = float()
cmd_vel_msg = Twist()
current_pose = Pose()
current_course = float()
goal_pose = Pose()
goal_pose_msg = Pose()

init_flag = False
finish_flag = True
goal_tolerance = 0.2
dist=0.0

#topics
cmd_vel_topic = "/velocity"
vel_topic = "/mavros/local_position/velocity"
goal_topic = "/route"
local_goal_topic = "/local_goal"
pose_topic = "/pose"
lidar_topic = "/scan"

#Obs_xy=[list(),list()]
Obs_xy=list()
lid_and_vec=list()
lid_ang_vec_new=list()
phi_new_vec=list()
phi_vec=list()
lidar_arr=list()
x_matrix=list()
y_matrix=list()
nearest_point= list()
i=0
j=0
k=0
Fatt=list()
yn=list()
xn=list()
xn_new=list()
yn_new=list()
phi_new_x=list()
lid_new_x=list()
phi_new_y=list()
lid_new_y=list()

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

def plan_virtual_fields():
    global goal_pose, goal_topic, first_waypoint_in_route, Frep, nearest_point, dist_goal, Obs_xy, dt, i, nearest_point, dist ,xn_new, yn_new, current_pose
    coordinates_obstacles()
    goal_new=Pose()
    #if (abs(dist) < goal_tolerance):
    #    Ev=0
    #    Ev_old=0
    #    sumEv=0
    #    Erot=0
    #    Erot_old=0
    #    sumErot=0
    #    finish_flag=True
    #else:
    Frep=[0,0]
    dist_goal = get_distance_to(current_pose, goal_pose)
    k=2
    goal_p=np.array([goal_pose.position.x,goal_pose.position.y])
    current_p=np.array([current_pose.position.x,current_pose.position.y])

    Fatt=k*(goal_p-current_p)/dist_goal
    print("goal_p",goal_p)
    print("current_p",current_p)
    print("Fatt", len(Fatt))
    print('xn_new_len',len(xn_new))
    if len(xn_new)!=0:
        nearest_point=None
        min_d = np.inf
        for i in range(len(xn_new)):
            d = math.sqrt((current_pose.position.x - xn_new[i]) ** 2 + (current_pose.position.y - yn_new[i]) ** 2)
                #print('min_d',min_d)
                #print('d =',d)
            if d < 1.5:
                if min_d > d:
                    min_d = d
                    nearest_point = [-xn_new[i],-yn_new[i]]
                    print("nearest_point",nearest_point)
        c=0.01
        if nearest_point!=None:
            current_po=[current_pose.position.x,current_pose.position.y]
            Frep_x=c*(current_po[0]-nearest_point[0])/min_d**2
            Frep_y=c*(current_po[1]-nearest_point[1])/min_d**2
            Frep=[Frep_x,Frep_y]


    r=4
    print("Frep =",len(Frep))
    print("Frep =", Frep)
    print("Fatt =", Fatt)

    x_new=current_pose.position.x+(Fatt[0]+Frep[0])*dt*r
    y_new=current_pose.position.y+(Fatt[1]+Frep[1])*dt*r

    goal_pose.position.x=x_new
    goal_pose.position.y=y_new
    xn_new=list()
    yn_new=list()
    print("goal_pose",goal_pose)
    return goal_pose

def coordinates_obstacles():
    global current_course, Obs_xy, lid_and_vec, lidar_arr, xn, yn, x_matrix, y_matrix, phi_new_vec, lid_ang_vec, phi_new_x, phi_new_y, lid_new_x, lid_new_y #, x_new, y_new
    alpha=math.radians(360)
    step=math.radians(1)
    lid = np.arange(-alpha/2,alpha/2+step,step)

    print("len_lidar",len(lidar_arr))
    print("lid_len",len(lid))
    lid_ang_vec =np.transpose(lid)
    phi_vec_1 = np.ones((alpha/step+1,1))*current_course
    phi_vec = np.transpose(phi_vec_1)
    phi_vec = phi_vec[0]
    x_matrix=np.ones((361,1))*current_pose.position.x
    y_matrix=np.ones((361,1))*current_pose.position.y
    print("phi_vec",len(phi_vec))
    print("lid_ang_vec",len(lid_ang_vec))
    print("lidar_arr_len=",len(lidar_arr))
    phi_new_x=list()
    phi_new_y=list()
    lid_new_x=list()
    lid_new_y=list()
    lidar_arr_new=list()
    #print("lidar_arr =",lidar_arr)
    for j in range(360):
        lidar_arr_new.append(lidar_arr[j])
        if len(lidar_arr_new)==360:
            lidar_arr_new.append(lidar_arr_new[0])
    print("len_lidar",len(lidar_arr_new))
    #print("lidar 361 =",lidar_arr_new)

    for j in range(len(phi_vec)):
        phi_new_y.append(math.sin(-phi_vec[j]+lid_ang_vec[j]))
        phi_new_x.append(math.cos(-phi_vec[j]+lid_ang_vec[j]))

        lid_new_x.append(lidar_arr_new[j]*phi_new_x[j])
        lid_new_y.append(lidar_arr_new[j]*phi_new_y[j])

    print('Phi_new_vec_x',len(phi_new_x))
    print('Phi_new_vec_y',len(phi_new_y))

    yn = lid_new_y + y_matrix[0]
    xn = lid_new_x + x_matrix[0]
    #xn_new=list()
    #yn_new=list()
    for i in range(len(xn)):
        if not np.isinf(xn[i]) and not np.isinf(yn[i]):
            xn_new.append(xn[i])
            yn_new.append(yn[i])
    print("xn =",len(xn_new))
    print("yn =",len(yn_new))
   # Obs_xy=[xn_new,yn_new]
    print(Obs_xy)
    #print("Obs_x",len(Obs_xy[0]))
   # print("Obs_y",len(Obs_xy[1]))

def goal_clb(data):
    #Get goal pose
    global goal_pose, init_flag, finish_flag
    if goal_pose!=0:
        rospy.sleep(0)
    goal_pose=data.pose
    init_flag = True
    finish_flag = False

def current_pose_clb(data):
    #Get current pose from topic
    global current_pose, current_course

    current_pose = data.pose
    rot = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    # # convert euler from quaternion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
    current_course = yaw

def laser_scan_clb(data):
    global lidar_arr
    lidar_arr=data.ranges

if __name__ == "__main__":

    # init ros node
    rospy.init_node('rc_potential_fields', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    # Get ros args
    if rospy.has_param('~goal_topic'):
        goal_topic = rospy.get_param('~goal_topic', goal_topic)
    if rospy.has_param('~pose_topic'):
        pose_topic = rospy.get_param('~pose_topic', pose_topic)

    # start subscriber
    #rospy.Subscriber(vel_topic, TwistStamped, vel_clb)
    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
    rospy.Subscriber(lidar_topic, LaserScan, laser_scan_clb)

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
                   # print("potential fields: not working")
                    currentTime =  0.0
                continue

            old_ros_time = rospy.get_time()
            goal_pose_msg = plan_virtual_fields() 
            if finish_flag:
                if currentTime > 1.0:
                    #print("pose controller: finish_flag True")
                    currentTime = 0.0
                cmd_vel_msg.linear.x = 0.0
                init_flag = False

             # publish msgs to the robot
            rate.sleep()

    except KeyboardInterrupt:   # if put ctr+c
        exit(0)




