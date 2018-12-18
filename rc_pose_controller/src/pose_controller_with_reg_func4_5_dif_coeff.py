#! /usr/bin/env python
# coding: utf-8

# This is pos controller for like-car robot


import math
import numpy as np
import rospy
import tf
import tf2_ros
import time
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import PointCloud2, LaserScan
from rc_bringup.cfg import PoseControllerConfig
from dynamic_reconfigure.server import Server
from pid_params_saver import YamlParams
from rc_car_msgs.msg import CarParams, CarPwmContol
from std_msgs.msg import Float64
#value
velocity = float()
cmd_vel_msg = Twist()
current_pose = Pose()
current_course = float()

goal_pose = Pose()
goal_pose_msg = Pose()
goal_new=Pose()
near_p=PoseStamped()
init_flag = False
max_vel = 1.1 # m/s
min_vel = -1.5 # m/s
max_angle = 25
ang_msg=Float64()
finish_flag = True
flag_back=False
goal_tolerance = 0.5
dist=0.0

#topics
cmd_vel_topic = "/cmd_vel"
vel_topic = "/mavros/local_position/velocity"
goal_topic = "/goal"
pose_topic = "/mavros/local_position/pose"
lidar_topic = "/scan"
point_cloud2_topic="/laserPointCLoud"

target_frame='odom'
source_frame='map'

#cfg_values
pps = YamlParams()
init_server = False

#PointCloud
max_dist_lidar=1.5
sonar_data = list()
#point_cloud = PointCloud2()
pc_msg = PointCloud2()
lp = lg.LaserProjection()

#PID
kP_pose=0.1
kI_pose=0.87
kD_pose=10

kP_course=0.5
kI_course=0.001
kD_course=0.0001

#reg_functions
v_des=0.0
Ev=0.0
Erot=0.0
upper_limit_of_ki_sum=0.0
lower_limit_of_ki_sum=0.0

u_v=0.0
u_rot=0.0
plot_x=[0]
plot_y=[0]
v=0.0
sumErot=0
sumEv=0
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
nearest_p=PoseStamped()
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
zeroth_val=Twist()
trial=int()
zero=CarPwmContol()
aproximation_point=None
point_cloud2=PointCloud2()

integrator=int()
#trap function for velocity
def trap_profile_linear_velocity(x, xy_des, v_max): 
    d = np.sqrt((x.x - xy_des.position.x) ** 2 + (x.y - xy_des.position.y) ** 2)
    if d <= 1:
        v_des = -d / (d - 1)
    else:
        v_des = v_max
    return v_des

#rotatin servo regulator
def rot_controller(Erot,Erot_old,sumErot,dT):
    global kP_course, kI_course, kD_course
    u_rot = kP_course * Erot + kI_course * sumErot + kD_course *(Erot-Erot_old) / dT
    return u_rot

#velocity motor regulator
def velocity_controller(Ev, Ev_old,sumEv, dT):
    global kP_pose, kI_pose, kD_pose
    u_v = kP_pose * Ev + kI_pose * sumEv + kD_pose *(Ev-Ev_old) / dT
    return u_v

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

def global2local(goal_pose,current_pose,current_course):
    loc=PoseStamped()
    loc.pose.position.x=(goal_pose.position.x-current_pose.position.x)*math.cos(current_course)+(goal_pose.position.y-current_pose.position.y)*math.sin(current_course)
    loc.pose.position.y=-(goal_pose.position.x-current_pose.position.x)*math.sin(current_course)+(goal_pose.position.y-current_pose.position.y)*math.cos(current_course)
    return loc

def coordinates_obstacles():
    global current_course, Obs_xy, lid_and_vec, lidar_arr, xn, yn, x_matrix, y_matrix, phi_new_vec, lid_ang_vec, phi_new_x, phi_new_y, lid_new_x, lid_new_y, lidar_arr_new, distance, nearest_p #, x_new, y_new
    alpha=math.radians(360)
    step=math.radians(1)
    lid = np.arange(-alpha/2,alpha/2+step,step)
    lid_ang_vec =np.transpose(lid)
    

    phi_new_x=list()
    phi_new_y=list()
    lid_new_x=list()
    lid_new_y=list()
    lidar_arr_new=list()
    xn_new=list()
    yn_new=list()
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
    min_i =  distance.index(min(distance))
    rotate=np.array([[math.cos(current_course),-math.sin(current_course)],
                     [math.sin(current_course),math.cos(current_course)]])
    nearest_point=np.array([[xn_new[min_i]],
                       [yn_new[min_i]]])
    nearest_p_in_global=np.dot(rotate,nearest_point)
    x=nearest_p_in_global[0]+current_pose.position.x
    y=nearest_p_in_global[1]+current_pose.position.y
    nearest_p.pose.position.x = x
    nearest_p.pose.position.y = y
    Obs_xy=[x,y]
    
    xn_new=list()
    yn_new=list()
    distance=list()
    return nearest_p



def plan_virtual_fields():
    global goal_pose, goal_topic,current_pose, first_waypoint_in_route, Frep, nearest_point, dist_goal, Obs_xy, dt, i, nearest_point, dist ,xn_new, yn_new, step_1, goal_new_p, Ev, Ev_old, sumEv, Erot, Erot_old, sumErot, finish_flag, range_dia, aproximation_point, lidar_arr_new, distance, point_cloud, Obs_xy
    
    #coordinates_obstacles()
    if get_distance_to(current_pose,goal_pose)>0.2 and step_1==1:
        if len(goal_new_p)<2:
            goal_new_p.append(goal_pose.position.x)
            goal_new_p.append(goal_pose.position.y)
    dist=np.sqrt((goal_new_p[0]-current_pose.position.x)**2+(goal_new_p[1]-current_pose.position.y)**2)
    

    if (abs(dist) < goal_tolerance):
        Ev=0
        Ev_old=0
        sumEv=0
        Erot=0
        Erot_old=0
        sumErot=0
        finish_flag=True
    
    Frep=[0,0]
    dist_goal = get_distance_to(current_pose, goal_pose)
    k=2

    goal_p=np.array([goal_pose.position.x,goal_pose.position.y])
    current_p=np.array([current_pose.position.x,current_pose.position.y])

    Fatt=k*(goal_new_p-current_p)/dist_goal #goal_p
    #print("goal_p",goal_p)
    #print("current_p",current_p)
    #print("Fatt", len(Fatt))
    #print('xn_new_len',len(xn_new))
    if len(xn_new)!=0:
        nearest_point=None
        min_d = np.inf
        d = math.sqrt((current_pose.position.x - Obs_xy[0]) ** 2 + (current_pose.position.y - Obs_xy[1]) ** 2)
        if d < 1.5:
            if (abs(current_course-(math.atan2(Obs_xy[1]-current_pose.position.y,Obs_xy[0]-current_pose.position.x))))<math.pi/2:
                if min_d > d:
                    min_d = d
                    nearest_point = [Obs_xy[0],Obs_xy[1]]
                   
                c=3 #0.01 10
                if nearest_point!=None:
                    current_po=[current_pose.position.x,current_pose.position.y]
                    Frep_x=c*(current_po[0]-nearest_point[0])/min_d**2
                    Frep_y=c*(current_po[1]-nearest_point[1])/min_d**2
                Frep=[Frep_x,Frep_y]

    r=1
    #print("nearest_point",nearest_point)
    #print("Frep =",len(Frep))
    #print("Frep =", Frep)
    #print("Fatt =", Fatt)

    x_new=current_pose.position.x+(Fatt[0]+Frep[0])*dt*r
    y_new=current_pose.position.y+(Fatt[1]+Frep[1])*dt*r

    goal_pose.position.x=x_new
    goal_pose.position.y=y_new
    xn_new=list()
    yn_new=list()
    distance=list()
    return goal_pose

def main():
    global dt, current_pose, current_course, goal_pose, cmd_vel_msg , u_v, u_rot, Ev, Erot,sumErot,sumEv, plot_x,plot_y, v_des, leinght_v,leinght_rot,v , finish_flag, goal_tolerance, dist, upper_limit_of_ki_sum, lower_limit_of_ki_sum, Ev_old, Erot_old, goal_new_p, point_cloud2, map, target_frame, source_frame
    loc=global2local(goal_pose,current_pose,current_course)
    #print("current_pose",current_pose)
   # print("goal_pose",goal_pose)
    v_des=trap_profile_linear_velocity(current_pose.position,goal_pose,max_vel)
    
    dx=(current_pose.position.x-plot_x[0])/dt
    
    dy=(current_pose.position.y-plot_y[0])/dt
    plot_x[0]=current_pose.position.x
    plot_y[0] = current_pose.position.y

    v = np.sqrt(dx**2+dy**2)
    if v!=0:
        if v>max_vel:
            v=max_vel
        if v<min_vel:
            v=min_vel

    Ev_old=Ev
    Ev=v_des-v
    sumEv=sumEv+Ev
    
    u_v=velocity_controller(Ev,Ev_old,sumEv,dt)
   # if (abs(current_course-(math.atan2(goal_pose.position.y-current_pose.position.y,goal_pose.position.x-current_pose.position.x))))>3*math.pi/6:
   #     u_v=-2.5

    u_v_constraints = [min_vel,max_vel]
    u_alpha_constraints=[-max_angle,max_angle]
    upper_limit_of_ki_sum=1.0
    lower_limit_of_ki_sum=-1.0
    #limit checker
    if u_v>u_v_constraints[1]:
        u_v = u_v_constraints[1]
    elif u_v<u_v_constraints[0]:
        u_v = u_v_constraints[0]
    Erot_old=Erot

    #Erot=np.arctan2(goal_pose.position.y-current_pose.position.y,goal_pose.position.x-current_pose.position.x)-current_course 
    Erot=np.arctan2(loc.pose.position.y,loc.pose.position.x)
    sumErot=sumErot+Erot

    u_rot = rot_controller(Erot,Erot_old,sumErot,dt)
    if u_rot>u_alpha_constraints[1]:
        u_rot=u_alpha_constraints[1]
    elif u_rot<u_alpha_constraints[0]:
        u_rot = u_alpha_constraints[0]
    if sumErot>=upper_limit_of_ki_sum:
        sumErot=upper_limit_of_ki_sum
    if sumErot<=lower_limit_of_ki_sum:
        sumErot=lower_limit_of_ki_sum
   # if (abs(current_course-(math.atan2(goal_pose.position.y-current_pose.position.y,goal_pose.position.x-current_pose.position.x))))>4*math.pi/6 and get_distance_to(goal_pose,current_pose)<=3:
   #     u_v=-1
    vel_and_angle=[u_v,u_rot]
    #print('u_v=',u_v)
    #output values of velocity and rotation
    vel_and_angle=[u_v,u_rot]
    cmd_vel_msg.linear.x=vel_and_angle[0]
   
    cmd_vel_msg.angular.z=vel_and_angle[1]

    return cmd_vel_msg


#def point_cloud2_clb(data):
#    global point_cloud2
#    point_cloud2=data
    #print(point_cloud2)

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


def cfg_callback(config, level):
    global max_vel, min_vel, max_angle, kP_pose, kI_pose, kD_pose, kP_course, kI_course, kD_course, init_server

    max_vel = float(config["max_vel"])
    min_vel = float(config["min_vel"])
    max_angle = math.radians(float(config["max_angle"]))
    kP_pose = float(config["kP_pose"])
    kI_pose = float(config["kI_pose"])
    kD_pose = float(config["kD_pose"])
    kP_course = float(config["kP_course"])
    kI_course = float(config["kI_course"])
    kD_course = float(config["kD_course"])
    if init_server:
        # opening pid params file
        pps.params_open()
        print ("open from cfg callback")

        # setting values
        pps.params_set('max_vel', config["max_vel"])
        pps.params_set('min_vel', config["min_vel"])
        pps.params_set('max_angle', config["max_angle"])
        pps.params_set('kP_pose', config["kP_pose"])
        pps.params_set('kI_pose', config["kI_pose"])
        pps.params_set('kD_pose', config["kD_pose"])
        pps.params_set('kP_course', config["kP_course"])
        pps.params_set('kI_course', config["kI_course"])
        pps.params_set('kD_course', config["kD_course"])


        # saving to file
        pps.params_save()
        print ("save from cfg callback")

    init_server = True

    return config



def set_server_value(cfg_srv):
    global max_vel, min_vel, max_angle, kP_pose,kI_pose,kD_pose,kP_course,kI_course,kD_course , init_server

    pps.params_open()
    print ("open from set server value")

    max_vel = pps.params_get('max_vel')
    min_vel = pps.params_get('min_vel')
    max_angle = pps.params_get('max_angle')
    kP_pose = pps.params_get('kP_pose')
    kI_pose = pps.params_get('kI_pose')
    kD_pose = pps.params_get('kD_pose')
    kP_course = pps.params_get('kP_course')
    kI_course = pps.params_get('kI_course')
    kD_course = pps.params_get('kD_course')


    cfg_srv.update_configuration({'max_vel': pps.params_get('max_vel'),
                                  'min_vel': pps.params_get('min_vel'),
                                  'max_angle': pps.params_get('max_angle'),
                                  'kP_pose': pps.params_get('kP_pose'),
                                  'kI_pose': pps.params_get('kI_pose'),
                                  'kD_pose': pps.params_get('kD_pose'),
                                  'kP_course': pps.params_get('kP_course'),
                                  'kI_course': pps.params_get('kI_course'),
                                  'kD_course': pps.params_get('kD_course')})
    pps.params_save()
    print ("save from set server value")

def callback(config):
   rospy.loginfo("Config set to {max_vel}".format(**config))
   rospy.loginfo("Config set to {min_vel}".format(**config))
   rospy.loginfo("Config set to {kP_pose}".format(**config))
   rospy.loginfo("Config set to {kI_pose}".format(**config))
   rospy.loginfo("Config set to {kD_pose}".format(**config))
   rospy.loginfo("Config set to {kP_course}".format(**config))
   rospy.loginfo("Config set to {kI_course}".format(**config))
   rospy.loginfo("Config set to {kD_course}".format(**config))

if __name__ == "__main__":


    # init ros node
    rospy.init_node('rc_pos_controller', anonymous=True)
    rate = rospy.Rate(20)  # 10hz

    # init dynamic reconfigure server
    cfg_srv = Server(PoseControllerConfig, cfg_callback)
    set_server_value(cfg_srv)
    # Get ros args
    if rospy.has_param('~vel_topic'):
        vel_topic = rospy.get_param('~vel_topic', vel_topic)
    if rospy.has_param('~cmd_vel'):
        cmd_vel_topic = rospy.get_param('~cmd_vel', cmd_vel_topic)
    if rospy.has_param('~goal_topic'):
        goal_topic = rospy.get_param('~goal_topic', goal_topic)
    if rospy.has_param('~pose_topic'):
        pose_topic = rospy.get_param('~pose_topic', pose_topic)

    if rospy.has_param('~max_vel'):
        max_vel = rospy.get_param('~max_vel', max_vel)
        cfg_srv.update_configuration({"max_vel": max_vel})
    if rospy.has_param('~min_vel'):
        min_vel = rospy.get_param('~min_vel', min_vel)
        cfg_srv.update_configuration({"min_vel": min_vel})
    if rospy.has_param('~max_angle'):
        max_angle = rospy.get_param('~max_angle', max_angle)
        cfg_srv.update_configuration({"max_angle": max_angle})
    ## PID params
    if rospy.has_param('~kP_pose'):  #pose means vel in case of repeatativenes but just now
        kP_pose = rospy.get_param('~kP_pose', kP_pose)
        cfg_srv.update_configuration({"kP_pose": kP_pose})
    if rospy.has_param('~kI_pose'):
        kI_pose = rospy.get_param('~kI_pose', kI_pose)
        cfg_srv.update_configuration({"kI_pose": kI_pose})
    if rospy.has_param('~kD_pose'):
        kD_pose = rospy.get_param('~kD_pose', kD_pose)
        cfg_srv.update_configuration({"kD_pose": kD_pose})
    if rospy.has_param('~kP_course'):
        kP_course = rospy.get_param('~kP_course', kP_course)
        cfg_srv.update_configuration({"kP_course": kP_course})
    if rospy.has_param('~kI_course'):
        kI_course = rospy.get_param('~kI_course', kI_course)
        cfg_srv.update_configuration({"kI_course": kI_course})
    if rospy.has_param('~kD_course'):
        kD_course = rospy.get_param('~kD_course', kD_course)
        cfg_srv.update_configuration({"kD_course": kD_course})
    # start subscriber
    #rospy.Subscriber(vel_topic, TwistStamped, vel_clb)
    rospy.Subscriber(goal_topic, PoseStamped, goal_clb)
    rospy.Subscriber(pose_topic, PoseStamped, current_pose_clb)
    rospy.Subscriber(lidar_topic, LaserScan, laser_scan_clb)

    vec_pub = rospy.Publisher(cmd_vel_topic, Twist,  queue_size=10)
    #new_goal_pub = rospy.Publisher(goal_topic, Pose, queue_size=10)
    near_pub= rospy.Publisher('nearest_point', PoseStamped, queue_size=10)
    zero_pub= rospy.Publisher("pwm",CarPwmContol, queue_size=10)
    ang_pub= rospy.Publisher("angle",Float64,queue_size=10)
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
            print('finish_flag',finish_flag)
            cmd_vel_msg = main()
            near_p = coordinates_obstacles()
           # goal_pose_msg = plan_virtual_fields()
            step_1=step_1+1
            if finish_flag:
                if currentTime > 1.0:
                    print("pose controller: finish_flag True")
                    currentTime = 0.0
                    goal_pose=current_pose
                    finish_flag=False
                #cmd_vel_msg.linear.x = 0.0
                init_flag = False

           # if flag_back==True:
           #     if cmd_vel_msg.linear.x<0:
           #         time.sleep(0.5)
           #         vec_pub.publish(cmd_vel_msg)
           #         print('1')
          # 	elif cmd_vel_msg.linear.x>0:
             #       print('2')
             #       flag_back=False
             #       integrator=0

            #if flag_back==False:
               # if cmd_vel_msg.linear.x<0:
               #     zeroth_val.linear.x=0
               #     vec_pub.publish(cmd_vel_msg)
               #     time.sleep(0.5)
               #     vec_pub.publish(zeroth_val)
               #     time.sleep(0.5)
               #     vec_pub.publish(cmd_vel_msg)
               #     trial+=1
               #     print('3')
               #     print("RGHAAAAAAAAAH")
               #     flag_back=True
                   # cmd_vel_msg.linear.x=0
                   # vec_pub.publish(cmd_vel_msg)
               # elif cmd_vel_msg.linear.x>0:
               #     print('4')
               #     vec_pub.publish(cmd_vel_msg)
            vec_pub.publish(cmd_vel_msg)
            ang_msg=current_course
            ang_pub.publish(current_course)
           
            near_pub.publish(near_p)
       #new_goal_pub.publish(goal_pose_msg) # publish msgs to the robot
            rate.sleep()
    except KeyboardInterrupt:   # if put ctr+c
        exit(0)






