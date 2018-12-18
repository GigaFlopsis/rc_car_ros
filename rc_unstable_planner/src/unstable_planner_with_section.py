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

#PID

#reg_functions
distance=list()

#obs_xy=[list(),list()]
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

ex_old=0
ey_old=0
kx=[0.5,0.001] #0.5,0.001
ky=[0.5,0.001]
cloud_of_np_global = list()
near_p=list()

aproximation_point=None
point_cloud2=PointCloud2()
#trap function for velocity


def coordinates_obstacles():
    global current_course, lid_and_vec, lidar_arr, xn, yn, x_matrix, y_matrix, phi_new_vec, lid_ang_vec, phi_new_x, phi_new_y, lid_new_x, lid_new_y, lidar_arr_new, distance, cloud_of_np_global,d,near_p
    cloud_of_np_global = list()
    alpha=math.radians(360)
    step=math.radians(1)
    lid = np.arange(-alpha/2,alpha/2+step,step)


    lid_ang_vec =np.transpose(lid)

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
    for j in range(len(lid_ang_vec)):
        phi_new_y.append(math.sin(lid_ang_vec[j])) #-phi_vec[j]))
        phi_new_x.append(math.cos(lid_ang_vec[j])) #-phi_vec[j]))

        lid_new_x.append(lidar_arr_new[j]*phi_new_x[j])
        lid_new_y.append(lidar_arr_new[j]*phi_new_y[j])


    yn = lid_new_y
    xn = lid_new_x 
    for i in range(len(xn)):
        if not np.isinf(xn[i]) and not np.isinf(yn[i]):
            xn_new.append(-xn[i])
            yn_new.append(-yn[i])

    for i in range(len(xn_new)):
        distance.append(math.sqrt((xn_new[i])**2+(yn_new[i])**2))
        if distance[i]<1:
            cloud_of_nearest_points.append(np.array([[xn_new[i]],[yn_new[i]]]))
    min_i =  distance.index(min(distance))
    d=distance[min_i]
    rotate=np.array([[math.cos(current_course),-math.sin(current_course)],
                     [math.sin(current_course),math.cos(current_course)]])
    
    for i in range(len(cloud_of_nearest_points)):
        cloud_rotated.append(np.dot(rotate,cloud_of_nearest_points[i]))
    
    for i in range(len(cloud_rotated)):
        cloud_of_np_global.append(np.array([[cloud_rotated[i][0]+current_pose.pose.position.x],[cloud_rotated[i][1]+current_pose.pose.position.y]]))





def goal_clb(data):
    #Get goal poserr
    global goal_pose, init_flag, finish_flag
    if goal_pose!=0:
        rospy.sleep(0)
    #print(type(goal_pose))
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

def unstable_planner2D(Obs_xy, r, goal, x, y, phi, planning_distance):
    global xn_new,yn_new,distance,goal_tolerance, finish_flag
    dist=np.sqrt((goal[0]-x)**2+(goal[1]-y)**2)

    if (abs(dist) < goal_tolerance):
        finish_flag=True
    beta_max=-100
    new_goal=PoseStamped()
    for i in range(len(Obs_xy)):
        rc=math.sqrt((x-Obs_xy[i][0])**2 +(y-Obs_xy[i][1])**2)
        beta=abs(rc-r)-(rc-r)
        if beta>0 and beta>beta_max:
            index_obs=i
            beta_max=beta
            rc_min=rc
    if beta_max!=-100:
        if len(goal)>2:
            drg=math.sqrt((x-goal[2])**2+(y-goal[3])**2)
        else:
            drg=math.sqrt((x-goal[0])**2+(y-goal[1])**2)
        if drg>rc_min:
            A=[0,0]
            B=[r,math.pi/2]
            dv=beta_max*(A[1]-B[1])/(A[0]-B[0])-A[0]*(A[1]-B[1])/(A[0]-B[0])+A[1]
            Ox=Obs_xy[index_obs][0]-x
            Oy=Obs_xy[index_obs][1]-y
            Rx=1*math.cos(phi)
            Ry=1*math.sin(phi)
            
            angle_robot_Obs=np.arccos((Ox*Rx+Oy*Ry)/(math.sqrt(Ox**2+Oy**2)*math.sqrt(Rx**2+Ry**2)))
            if (abs(angle_robot_Obs)<math.pi/2):
                kvec=Rx*Oy-Ry*Ox
                if kvec==0:
                    virtual_angle=phi+(np.sign([-1.0+2.0*np.random.randn()]))*dv
                elif kvec>0:
                    virtual_angle=phi-dv
                elif kvec<0:
                    virtual_angle=phi+dv
            else:
                virtual_angle=phi
            goal=np.array([[x],[y]])+5*np.array([[math.cos(virtual_angle)],[math.sin(virtual_angle)]])
    res=PI_planner2D(current_pose.pose.position.x,current_pose.pose.position.y,goal,planning_distance)
    new_goal.pose.position.x=res[0]
    new_goal.pose.position.y=res[1]
    new_goal.pose.orientation.w=res[2]
    print("new_goal",new_goal)

    Obs_xy=list()
    xn_new=list()
    yn_new=list()
    distance=list()
    return new_goal

def PI_planner2D(x,y,goal,planning_distance):
    global ex_old,ey_old, kx, ky
    if len(goal)==2:
        ex=goal[0]-x
        ey=goal[1]-y
    else:
        coeff_line2_res=coeffs_line2(goal)
        A=coeff_line2_res[0]
        B=coeff_line2_res[1]
        C=coeff_line2_res[2]
        coeffs_normal_res=coeffs_normal(x,y,A,B)
        A1=coeffs_normal_res[0]
        B1=coeffs_normal_res[1]
        C1=coeffs_normal_res[2]
        intersection_line_normal_res=intersection_line_normal(A,B,C,A1,B1,C1)
        xr=intersection_line_normal_res[0]
        yr=intersection_line_normal_res[1]
        ex=goal[2]-x
        ey=goal[3]-y
        if (planning_distance<math.sqrt((x-goal[2])**2+(y-goal[3])**2)):
            find_next_point_res=find_next_point(xr,yr,goal[2],goal[3],planning_distance)
            xn=find_next_point_res[0]
            yn=find_next_point_res[1]
            ex=xn-x
            ey=yn-y
    x_plan=x+kx[0]*ex+kx[1]*(ex+ex_old)
    y_plan=y+ky[0]*ey+kx[1]*(ey+ey_old)
    phi_plan=math.atan2(y_plan,x_plan)
    coeffs=[x_plan,y_plan,phi_plan,ex,ey]
    return coeffs

def coeffs_line2(R):
    x1=R[0]
    y1=R[1]
    x2=R[2]
    y2=R[3]
    B=None
    if (x2-x1)==0:
        A=1
        B=0
    else:
        A=1/(x2-x1)
    if y2-y1==0:
        B=1
        A=0
    elif np.nan(B):
        B=-1/(y2-y1)
    C=y1*(-B)-x1*A
    coeffs=[A,B,C]
    return coeffs

def coeffs_normal(x1,y1,A,B):
    A1=B
    B1=-A
    C1=y1*(-B1)-x1*A1
    coeffs=[A1,B1,C1]
    return coeffs

def intersection_line_normal(A1,B1,C1,A2,B2,C2):
    if A1==0 and B2!=0:
        yr=(0-C2)/(B2-0)
        xr=0
    elif A1==0 and B2==0:
        yr=-C1
        xr=-C2
    elif (B2-A2*B1/A1)==0:
        yr=0
        xr=(-C1*B1*yr)/A1
    else:
        yr=(A2*C1/A1-C2)/(B2-A2*B1/A1)
        xr=(-C1-B1*yr)/A1
    coeffs=[xr,yr]
    return coeffs

def find_next_point(x1,y1,x2,y2,planning_distance):
    Rab=math.sqrt((x2-x1)**2+(y2-y1)**2)
    k=planning_distance/Rab
    xn=x1+(x2-x1)*k
    yn=y1+(y2-y1)*k
    coeffs=[xn,yn]
    return coeffs


if __name__ == "__main__":


    # init ros node
    rospy.init_node('potential_fields', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

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
            goal_new_p=[1,1,2.5,3]
            old_ros_time = rospy.get_time()
            #goal_pose_msg = plan_virtual_fields()
            coordinates_obstacles()
            goal_pose_msg= unstable_planner2D(cloud_of_np_global,1, goal_new_p, current_pose.pose.position.x, current_pose.pose.position.y, current_course, 1)
            
            if finish_flag:
                if currentTime > 1.0:
                    print("pose controller: finish_flag True")
                    step_1=0
                    currentTime = 0.0
                    cmd_vel_msg.linear.x=0
                    #vec_pub.publish(cmd_vel_msg)
                    init_flag = False
           
            new_goal_pub.publish(goal_pose_msg)
            rate.sleep()

    except KeyboardInterrupt:   # if put ctr+c
        exit(0)

