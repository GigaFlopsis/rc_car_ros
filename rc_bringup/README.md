# The main ROS node for autonomous control of the robot car
These methods do not pretend to be original, most of it I borrowed from the project of the [Erle rover](http://erlerobotics.com/blog/erle-rover/)


## Launch files
1. **bringup.launch**	- Launches main modules for work in autonomous mode 
2. **tf.launch**	- setup tf
3. **mapping_default.launch**	- Hector slam node for created map  
4. **navigation.launch**- Launch costmap and move_base ("move_base" yet disabled for debugging)
5. **localizacion.launch** - AMCL (include of navigation.launch)
6. **move_base.launch**	- move_base (include of navigation.launch)
7. **odom_laser.launch**	- get tf transform of lidar
8. **rplidar.launch**	- run lidar
9. **view_navigation.launch**	- open rviz (for desktop)

## Nodes:

### 1. Remote control
**file:** rc_control.py<br/>
**Description:** The pyhton script for remote control car which is directly connected to the PWM outputs pin of the RPI.<br/>
Ideally, you need to adjust the input data so that it matches the actual speed, this can be done by finding the relationship between the speed of the car and the output pulse.<br/>
**input type**: [geometry_msgs:Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)<br/>
where:

```
linear:
    x: forwand + / - backward 
    y: 0.0
    z: 0.0
  angular:
    x: 0.0
    y: 0.0
    z: rotate
```
**input type**: PWM pulse
