# The main ROS node for autonomous control of the robot car
These methods do not pretend to be original, most of it I borrowed from the project of the [Erle rover](http://erlerobotics.com/blog/erle-rover/)


## Launch files
1. **bringup.launch**	- Launches main modules for work in autonomous mode 
2. **bringup_no_map.launch**	- Launches main modules for work in autonomous mode without a map
3. **create_map.launch** - Launches for create map via hector slam
4. **tf.launch**	- setup tf
5. **mapping_default.launch**	- Hector slam node for created map
6. **localizacion.launch** - AMCL (include of navigation.launch)
7. **move_base.launch**	- the base controller, uses [dwa_local_planner](http://wiki.ros.org/dwa_local_planner) (by default included in bringup.launch)
8. **teb_move_base.launch**	- the base controller, uses [teb_local_planner](http://wiki.ros.org/teb_local_planner)
9. **odom_laser.launch**	- get tf transform of lidar
10. **rplidar.launch**	- the prlidar driver
11. **view_navigation.launch**	- open rviz (for desktop) if you use move_base.launch
12. **view_navigation_teb.launch**	- open rviz (for desktop) if you use teb_move_base.launch


## Nodes:

### 1. rc_control.py<br/>
**Description:** The pyhton script for remote control car which is directly connected to the PWM outputs pin of the RPI.<br/>
Ideally, you need to adjust the input data so that it matches the actual speed, this can be done by finding the relationship between the speed of the car and the output pulse.<br/>

#### Subscribed Topics:
/cmd_vel ([geometry_msgs:Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))<br/>
```
linear:
    x: forwand + / - backward velocity
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: steering_angle
```

/pwm ([rc_bringup:CarPwmContol](https://github.com/GigaFlopsis/rc_car_ros/blob/master/rc_bringup/msg/CarPwmContol.msg))<br/>

```
motor: pwm
servo: pwm
```
/ackermann_cmd ([ackermann_msgs:AckermannDriveStamped](http://docs.ros.org/api/ackermann_msgs/html/msg/AckermannDriveStamped.html))<br/>

```
linear:
    x: forwand + / - backward velocity
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: steering_angle_velocity
```
#### Publisher:

/pwm_output ([rc_bringup:CarPwmContol](https://github.com/GigaFlopsis/rc_car_ros/blob/master/rc_bringup/msg/CarPwmContol.msg))<br/>
**Output param**: PWM pulse

```
motor: pwm
servo: pwm
```

/params ([rc_car_msgs:CarParams](https://github.com/GigaFlopsis/rc_car_ros/blob/master/rc_car_msgs/msg/CarParams.msg))<br/>
&emsp;&emsp; Output param: PWM pulse


#### Service:

/car/set_mode ([std_srvs:SetBool](http://docs.ros.org/kinetic/api/std_srvs/html/srv/SetBool.html))<br/>
*Enable / Disable motor.<br/>*

```
bool: data     # on /off motor
```

/params ([rc_car_msgs:CarParams](https://github.com/GigaFlopsis/rc_car_ros/blob/master/rc_car_msgs/msg/CarParams.msg))<br/>
**Output param**: PWM pulse

#### Parameters:

#### 1. Controller params:
~use_imu_vel(bool, default: False)<br/>
&emsp;&emsp;*Use real velocity data from regulator control.<br/>*
~max_vel (float, default: "1.0")<br/>
&emsp;&emsp;*The maximum speed at which the car moves (This is a relative parameter that is configurable by trial).<br/>*
~min_vel (float, default: "1.0")<br/>
&emsp;&emsp;*The minimum speed at which the car moves (This is a relative parameter that is configurable by trial).<br/>*
~wheelbase (float, default: "0.28")<br/>
&emsp;&emsp;*The length wheelbase of car in meters.<br/>*
~ max_angle (float, default: "25.0")<br/>
&emsp;&emsp;*The max steering angle of car in degrees.<br/>*


#### 2. GPIO params:
~servo_pin (int, default:"4")<br/>
&emsp;&emsp;*The pin out of servo PWM<br/>*
~middle_servo (int, default:"1500")<br/>
&emsp;&emsp;*The midle position of servo<br/>*
~servo_offset (int, default:"47")<br/>
&emsp;&emsp;*The offset of servo for to correct the middle pose<br/>*
~motor_pin (int, default:"7")<br/>
&emsp;&emsp;*The pin out of motor PWM<br/>*
~middle_motor (int, default:"1550")<br/>
&emsp;&emsp;*Zero position of motor<br/>*
~revers_servo (bool, default:"False")<br/>
&emsp;&emsp;*Rivers of servo direction<br/>*


#### 2. Topics setup:
~cmd_vel (string, default: "cmd_vel")<br/>
&emsp;&emsp;*The vel topic of subscribed for remote via velocity.<br/>*
~drive_topic(string, default: "ackermann_cmd")<br/>
&emsp;&emsp;*The remote topic of subscribed for remote like-car.<br/>*
~pwm_topic (string, default: "pwm")<br/>
&emsp;&emsp;*The pwm topic of subscribed for direct remote of PWM.<br/>*
~pwm_output_topic (string, default: "pwm_output")<br/>
&emsp;&emsp;*The topic publishes a signal on the motors.<br/>*
~vel_topic (string, default: "/mavros/local_position/velocity")*<br/>
&emsp;&emsp;*The vel topic of subscribed for get velocity from IMU.*<br/> 
&emsp;&emsp;*P.S.: by default mavros send velocity data from NED coords.<br/>*
~param_topic(string, default: "/params")*<br/>
&emsp;&emsp; The topic publishes a car parameters<br/>

### 2. tf_to_vel.py<br/>
**Description:** The pyhton node for get linear speed from TF.<br/>

#### Subscribed Topics:
tf([tf/tfMessage.msg](http://docs.ros.org/api/tf/html/msg/tfMessage.html))<br/>

#### Publisher Topics:
velocity [geometry_msgs:TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html)<br/>

#### Parameters:
~vel_topic (string, default: "velocity")<br/>
&emsp;&emsp;*Topic for publication.<br/>*
~base_link (string, default: "map")<br/>
&emsp;&emsp;*The perant tf.<br/>*
~child_link (string, default: "base_link")<br/>
&emsp;&emsp;*The child tf.<br/>*


### 3. pose_controller.py<br/>
**Description:** The position controller. Ensures the movement of the car in the goal point (Well, takes into account the orientation at a goal point).<br/>

#### Subscribed Topics:
tf([tf/tfMessage.msg](http://docs.ros.org/api/tf/html/msg/tfMessage.html))<br/>
vel_topic ([geometry_msgs:TwistStamped](http://docs.ros.org/api/geometry_msgs/html/msg/TwistStamped.html))<br/>
move_base_simple/goal ([geometry_msgs:PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))<br/>

#### Publisher Topics:
cmd_vel ([geometry_msgs:Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html))<br/>

#### Parameters:

Also PID parameters are available through [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure)

~cmd_vel (string, default: "cmd_vel")<br/>
&emsp;&emsp;*Name of topic for set control.<br/>*
~vel_topic (string, default: "velocity")<br/>
&emsp;&emsp;*Name of topic for get velocity data .<br/>*
~goal_topic (string, default: "move_base_simple/goal")<br/>
&emsp;&emsp;*Name of topic for get goal point.<br/>*
~base_link (string, default: "map")<br/>
&emsp;&emsp;*The perant tf.<br/>*
~child_link (string, default: "base_link")<br/>
&emsp;&emsp;*The child tf.<br/>*

~max_vel (float, default: "1.0")<br/>
&emsp;&emsp;*The maximum output velocity from controller (in m/s).<br/>*
~min_vel (float, default: "-1.5")<br/>
&emsp;&emsp;*The minimum output velocity from controller (in m/s).<br/>*
~max_angle (float, default: "25.0")<br/>
&emsp;&emsp;*The maximum output angle from controller (in degrees).<br/>*
~goal_tolerance (float, default: "0.4")<br/>
&emsp;&emsp;*The maximum  tolerance to goal point (in meters).<br/>*

~kP_pose (float, default: "1.0")<br/>
&emsp;&emsp;*The "P" coefficient for XY of PID controller.<br/>*
~kI_pose (float, default: 0.0")<br/>
&emsp;&emsp;*The "I" coefficient for XY of PID controller.<br/>*
~kD_pose (float, default: 0.2")<br/>
&emsp;&emsp;*The "D" coefficient for XY of PID controller.<br/>*

~kP_course (float, default: "0.5")<br/>
&emsp;&emsp;*The "P" coefficient for course of PID controller.<br/>*
~kI_course (float, default: "0.0")<br/>
&emsp;&emsp;*The "I" coefficient for course of PID controller.<br/>*
~kD_course (float, default: "0.2")<br/>
&emsp;&emsp;*The "D" coefficient for course of PID controller.<br/>*
