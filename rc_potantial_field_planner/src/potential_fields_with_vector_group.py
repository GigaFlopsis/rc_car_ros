#! /usr/bin/env python
# coding: utf-8

# This is pos controller for like-car robot


import math
import numpy as np
import rospy
import tf
import tf2_ros
#import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
#from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan

#value
velocity = float()
cmd_vel_msg = Twist()
current_pose = PoseStamped()
current_course = float()

goal_pose = PoseStamped()
goal_pose_msg = PoseStamped()
goal_new=PoseStamped()

init_flag = False
max_vel = 1.1 # m/s
min_vel = -1.5 # m/s
max_angle = 25

finish_flag = True
goal_tolerance = 0.5
dist=0.0

#topics
cmd_vel_topic = "/cmd_vel"
vel_topic = "/mavros/goal/velocity"
goal_topic = "/goal"
local_goal_topic = "/local_goal"
pose_topic = "/pose"
lidar_topic = "/scan"
point_cloud2_topic="/laserPointCLoud"

target_frame='odom'
source_frame='map'

#cfg_values
init_server = False

#PointCloud
max_dist_lidar=1.5
sonar_data = list()
#point_cloud = PointCloud2()
pc_msg = PointCloud2()
lp = lg.LaserProjection()

#PID

#reg_functions
distance=list()

#obs_xy=[list(),list()]
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
range_dia=None
goal_new_p=list()
step_1=1

aproximation_point=None
point_cloud2=PointCloud2()
#trap function for velocity


def coordinates_obstacles():
    global current_course, Obs_xy, lid_and_vec, lidar_arr, xn, yn, x_matrix, y_matrix, phi_new_vec, lid_ang_vec, phi_new_x, phi_new_y, lid_new_x, lid_new_y, lidar_arr_new, distance, cloud_of_np_global,d 
    alpha=math.radians(360)
    step=math.radians(1)
    lid = np.arange(-alpha/2,alpha/2+step,step)
 
    print("len_lidar",len(lidar_arr))
    print("lid_len",len(lid))
    lid_ang_vec =np.transpose(lid)
    print("lid_ang_vec",len(lid_ang_vec))
    cloud_of_nearest_points=list()
    cloud_rotated=list()
    cloud_of_np_global=list()
    phi_new_x=list()
    phi_new_y=list()
    lid_new_x=list()
    lid_new_y=list()
    lidar_arr_new=list()
    for j in range(360):
        lidar_arr_new.append(lidar_arr[j])
        if len(lidar_arr_new)==360:
            lidar_arr_new.append(lidar_arr_new[0])
    print("len_lidar",len(lidar_arr_new))
    for j in range(len(lid_ang_vec)):
        phi_new_y.append(math.sin(lid_ang_vec[j])) #-phi_vec[j]))
        phi_new_x.append(math.cos(lid_ang_vec[j])) #-phi_vec[j]))

        lid_new_x.append(lidar_arr_new[j]*phi_new_x[j])
        lid_new_y.append(lidar_arr_new[j]*phi_new_y[j])

    print('Phi_new_vec_x',len(phi_new_x))
    print('Phi_new_vec_y',len(phi_new_y))

    yn = lid_new_y
    xn = lid_new_x 
    for i in range(len(xn)):
        if not np.isinf(xn[i]) and not np.isinf(yn[i]):
            xn_new.append(-xn[i])
            yn_new.append(-yn[i])
    print("xn =",len(xn_new))
    print("yn =",len(yn_new))
    for i in range(len(xn_new)):
        distance.append(math.sqrt((xn_new[i])**2+(yn_new[i])**2))
        if distance[i]<0.5:
            cloud_of_nearest_points.append(np.array([[xn_new[i]],[yn_new[i]]]))
    min_i =  distance.index(min(distance))
    d=distance[min_i]
    rotate=np.array([[math.cos(current_course),-math.sin(current_course)],
                     [math.sin(current_course),math.cos(current_course)]])
    
    for i in range(len(cloud_of_nearest_points)):
        cloud_rotated.append(np.dot(rotate,cloud_of_nearest_points[i]))
    
    for i in range(len(cloud_rotated)):
        cloud_of_np_global.append(np.array([[cloud_rotated[i][0]+current_pose.pose.position.x],[cloud_rotated[i][1]+current_pose.pose.position.y]]))
    print('cloud_of_np_global',cloud_of_np_global)
    #nearest_p_in_global=np.dot(rotate,nearest_point)
    #x=nearest_p_in_global[0]+current_pose.position.x
    #y=nearest_p_in_global[1]+current_pose.position.y
    #x=cloud_of_np_global[0][0]
    #y=cloud_of_np_global[0][1]
    #Obs_xy=[x,y]
    #print("Obs_xy",Obs_xy)

def plan_virtual_fields():
    global goal_pose, goal_topic,current_pose, first_waypoint_in_route, Frep, nearest_point, dist_goal, Obs_xy, dt, i, nearest_point, dist ,xn_new, yn_new, step_1, goal_new_p, Ev, Ev_old, sumEv, Erot, Erot_old, sumErot, finish_flag, range_dia, aproximation_point, lidar_arr_new, distance, point_cloud, Obs_xy, cloud_of_np_global,d
 
    
    coordinates_obstacles()
    if (np.sqrt((goal_pose.pose.position.x-current_pose.pose.position.x)**2+(goal_pose.pose.position.y-current_pose.pose.position.y)**2))>0.2 and step_1==1:
        if len(goal_new_p)<2:
            goal_new_p.append(goal_pose.pose.position.x)
            goal_new_p.append(goal_pose.pose.position.y)
    dist=np.sqrt((goal_new_p[0]-current_pose.pose.position.x)**2+(goal_new_p[1]-current_pose.pose.position.y)**2)
    print("dist= ",dist)

    if (abs(dist) < goal_tolerance):
        Ev=0
        Ev_old=0
        sumEv=0
        Erot=0
        Erot_old=0
        sumErot=0
        finish_flag=True
    #else:
    print("step_1",step_1)
    print("goal_new_p",goal_new_p)
    Frep=[0,0]
    dist_goal = np.sqrt((-goal_new_p[0]+current_pose.pose.position.x)**2+(-goal_new_p[1]+current_pose.pose.position.y)**2)
    k=1 #goal_pose

    goal_p=np.array([goal_pose.pose.position.x,goal_pose.pose.position.y])
    current_p=np.array([current_pose.pose.position.x,current_pose.pose.position.y])

    Fatt=k*(goal_new_p-current_p)/dist_goal #goal_p
    print("goal_p",goal_p)
    print("current_p",current_p)
    print("Fatt", len(Fatt))
    print('xn_new_len',len(xn_new))
    if len(xn_new)!=0:
        nearest_point=None
        Frep_x_prev=0
        Frep_y_prev=0
        Frep_xn=0
        Frep_yn=0
        min_d = np.inf
        #d = math.sqrt((current_pose.position.x - Obs_xy[0]) ** 2 + (current_pose.position.y - Obs_xy[1]) ** 2)
        print("d=",d)
        if d < 0.5:

            #if (abs(current_course - (math.atan2(Obs_xy[1] - current_pose.pose.position.y,Obs_xy[0] - current_pose.pose.position.x)))) < math.pi / 2:
            if min_d > d:
                min_d = d
                nearest_point =1

            c=0.04 #0.01 10
            if nearest_point!=None:
                current_po=[current_pose.pose.position.x,current_pose.pose.position.y]
                for i in range(len(cloud_of_np_global)):
                    Frep_xn = (current_po[0]-cloud_of_np_global[i][0])+Frep_x_prev
                    Frep_yn = (current_po[1]-cloud_of_np_global[i][1])+Frep_y_prev
                    Frep_x_prev = Frep_xn
                    Frep_y_prev = Frep_yn
            Frep_x = (c*(Frep_xn)/min_d**2)
            Frep_y = (c*(Frep_yn)/min_d**2)
            Frep=[Frep_x,Frep_y]

    r=1
    print("nearest_point",nearest_point)
    print("Frep =",len(Frep))
    print("Frep =", Frep)
    print("Fatt =", Fatt)

    x_new=current_pose.pose.position.x+(Fatt[0]+Frep[0])*dt*r
    y_new=current_pose.pose.position.y+(Fatt[1]+Frep[1])*dt*r

    goal_pose.pose.position.x=x_new
    goal_pose.pose.position.y=y_new
    xn_new=list()
    yn_new=list()
    distance=list()
    print('goal_pose',goal_pose)
    return goal_pose



def goal_clb(data):
    #Get goal poserr
    global goal_pose, init_flag, finish_flag
    if goal_pose!=0:
        rospy.sleep(0)
    print(type(goal_pose))
    goal_pose=data
    init_flag = True
    finish_flag = False

def current_pose_clb(data):
    #Get current pose from topic
    global current_pose, current_course

    current_pose = data
    rot = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    # # convert euler from quaternion
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
    current_course = yaw

def laser_scan_clb(data):
    global lidar_arr
    lidar_arr=data.ranges


if __name__ == "__main__":


    # init ros node
    rospy.init_node('potential_fields', anonymous=True)
    rate = rospy.Rate(5)  # 10hz

    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
    rospy.Subscriber(lidar_topic, LaserScan, laser_scan_clb)
    vec_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    new_goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=10)
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
                    #print("pose controller: not init")
                    currentTime =  0.0
                continue

            old_ros_time = rospy.get_time()

            goal_pose_msg = plan_virtual_fields()
            step_1=step_1+1
            if finish_flag:
                if currentTime > 1.0:
                    print("pose controller: finish_flag True")
                    currentTime = 0.0
                    #vec_pub.publish(cmd_vel_msg)


                init_flag = False
            new_goal_pub.publish(goal_pose_msg)
            rate.sleep()

    except KeyboardInterrupt:   # if put ctr+c
        exit(0)
