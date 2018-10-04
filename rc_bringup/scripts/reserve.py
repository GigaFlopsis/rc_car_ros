#! /usr/bin/env python
# coding: utf-8

# This is pos controller for like-car robot


import math
import numpy as np
import rospy
import tf
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from rc_bringup.cfg import PoseControllerConfig
from dynamic_reconfigure.server import Server
from pid_params_saver import YamlParams

#value
velocity = float()
cmd_vel_msg = Twist()
current_pose = Pose()
current_course = float()

goal_pose = Pose()
goal_pose_msg = Pose()
goal_new=Pose()

init_flag = False
max_vel = 1.1 # m/s
min_vel = -1.5 # m/s
max_angle = 25

finish_flag = True
goal_tolerance = 1
dist=0.0

#topics
cmd_vel_topic = "/cmd_vel"
vel_topic = "/mavros/local_position/velocity"
goal_topic = "/goal"
local_goal_topic = "/local_goal"
pose_topic = "/mavros/local_position/pose"
lidar_topic = "/scan"
#cfg_values
pps = YamlParams()
init_server = False

#PID
kP_pose=float
kI_pose=float
kD_pose=float

kP_course=float
kI_course=float
kD_course=float

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
distance=0

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
range_dia=None
goal_new_p=list()
step_1=1

aproximation_point=None

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
    kP_course = 0.435 #0.9
    kI_course = 0.0001 #0.00258
    kD_course = 0.00001 #0.00477 
    u_rot = kP_course * Erot + kI_course * sumErot + kD_course *(Erot-Erot_old) / dT
    return u_rot

#velocity motor regulator
def velocity_controller(Ev, Ev_old,sumEv, dT):
    global kP_pose, kI_pose, kD_pose
    kP_pose = 0.1
    kI_pose= 0.87
    kD_pose= 0.001
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


def plan_virtual_fields():
    global goal_pose, goal_topic, first_waypoint_in_route, Frep, nearest_point, dist_goal, Obs_xy, dt, i, nearest_point, dist ,xn_new, yn_new, step_1, goal_new_p, Ev, Ev_old, sumEv, Erot, Erot_old, sumErot, finish_flag, range_dia, aproximation_point

    coordinates_obstacles()
    if get_distance_to(current_pose,goal_pose)>0.2 and step_1==1:
        if len(goal_new_p)<2:
            goal_new_p.append(goal_pose.position.x)
            goal_new_p.append(goal_pose.position.y)
    dist=np.sqrt((goal_new_p[0]-current_pose.position.x)**2+(goal_new_p[1]-current_pose.position.y)**2)
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
    dist_goal = get_distance_to(current_pose, goal_pose)
    k=2
    goal_p=np.array([goal_pose.position.x,goal_pose.position.y])
    current_p=np.array([current_pose.position.x,current_pose.position.y])

    Fatt=k*(goal_new_p-current_p)/dist_goal #goal_p
    #Fatt=k*(nearest_point-current_p)/dist_goal
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
                   # if aproximation_point!=None: 
                   #     range_dia=math.sqrt((aproximation_point[0]-nearest_point[0])**2+(aproximation_point[1]-nearest_point[1])**2)
                   #     if range_dia<0.03:
                   #         nearest_point=aproximation_point

                   
        c=2 #0.01 10
        if nearest_point!=None:
            current_po=[current_pose.position.x,current_pose.position.y]
            Frep_x=c*(current_po[0]-nearest_point[0])/min_d**2
            Frep_y=c*(current_po[1]-nearest_point[1])/min_d**2
            Frep=[Frep_x,Frep_y]


    r=1 #10
    print("nearest_point",nearest_point)
    print("Frep =",len(Frep))
    print("Frep =", Frep)
    print("Fatt =", Fatt)

    x_new=current_pose.position.x+(Fatt[0]+Frep[0])*dt*r
    y_new=current_pose.position.y+2*(Fatt[1]+Frep[1])*dt*r

    goal_pose.position.x=x_new
    goal_pose.position.y=y_new
    xn_new=list()
    yn_new=list()

    #if (abs(dist) < goal_tolerance):
    #    Ev=0
    #    Ev_old=0
    #    sumEv=0
    #    Erot=0
    #    Erot_old=0
    #    sumErot=0
    #    finish_flag=True

   # aproximation_point=nearest_point
    print("goal_new",goal_pose)
    return goal_pose

def main():
    global dt, current_pose, current_course, goal_pose, cmd_vel_msg , u_v, u_rot, Ev, Erot,sumErot,sumEv, plot_x,plot_y, v_des, leinght_v,leinght_rot,v , finish_flag, goal_tolerance, dist, upper_limit_of_ki_sum, lower_limit_of_ki_sum, Ev_old, Erot_old, goal_new_p
    print("current_pose",current_pose)
    print("goal_pose",goal_pose)
    #dist=np.sqrt((goal_new_p[0]-current_pose.position.x)**2+(goal_new_p[1]-current_pose.position.y)**2)
    #car brake and PID reconfiguration to zero after destination point

    v_des=trap_profile_linear_velocity(current_pose.position,goal_pose,max_vel)
    dx=(current_pose.position.x-plot_x[0])/dt
    
    dy=(current_pose.position.y-plot_y[0])/dt
    plot_x[0]=current_pose.position.x
    plot_y[0] = current_pose.position.y
    #general velocity
    v = np.sqrt(dx**2+dy**2)
    Ev_old=Ev
    Ev=v_des-v
    sumEv=sumEv+Ev
    
    u_v=velocity_controller(Ev,Ev_old,sumEv,dt)
    #constraints for lower and upper limits
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

    Erot=np.arctan2(goal_pose.position.y-current_pose.position.y,goal_pose.position.x-current_pose.position.x)-current_course 
    sumErot=sumErot+Erot
    #if (abs(np.degrees(Erot))>90):
    #    u_v=min_vel

    u_rot = rot_controller(Erot,Erot_old,sumErot,dt)
    if u_rot>u_alpha_constraints[1]:
        u_rot=u_alpha_constraints[1]
    elif u_rot<u_alpha_constraints[0]:
        u_rot = u_alpha_constraints[0]
    if sumErot>=upper_limit_of_ki_sum:
        sumErot=upper_limit_of_ki_sum
    if sumErot<=lower_limit_of_ki_sum:
        sumErot=lower_limit_of_ki_sum
    vel_and_angle=[u_v,u_rot]
    #output values of velocity and rotation
    vel_and_angle=[u_v,u_rot]
    cmd_vel_msg.linear.x=vel_and_angle[0]
    #print(u_v)
    cmd_vel_msg.angular.z=vel_and_angle[1]

    return cmd_vel_msg

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
    global max_vel, min_vel, max_angle, init_server

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
    global max_vel, min_vel, max_angle, init_server

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

if __name__ == "__main__":

    # init ros node
    rospy.init_node('rc_pos_controller', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

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
    new_goal_pub = rospy.Publisher(goal_topic, Pose, queue_size=10)

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

            cmd_vel_msg = main()
            goal_pose_msg = plan_virtual_fields()
            step_1=step_1+1 
            if finish_flag:
                if currentTime > 1.0:
                    print("pose controller: finish_flag True")
                    currentTime = 0.0
                cmd_vel_msg.linear.x = 0.0
                init_flag = False

            vec_pub.publish(cmd_vel_msg)
            new_goal_pub.publish(goal_pose_msg) # publish msgs to the robot
            rate.sleep()

    except KeyboardInterrupt:   # if put ctr+c
        exit(0)




