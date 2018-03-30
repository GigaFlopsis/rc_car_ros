# Autonomous RC vehicle for ROS
### [ROS](http://www.ros.org/) package to control an autonomous RC vehicle based on Raspberry Pi3.

This development includes of methods self-driving in indoor environment with used SLAM navigation [rplidar A2](https://www.slamtec.com/en/Lidar/A2#)
 > **Authors:** Dmitry Devitt, Konovalov Georgy<br/>
 > **Maintainer:** Dmitry Devitt, devittdv@gmail.com <br/>
 > **Affiliation:** [Raccoonlab](http://Raccoonlab.org), [Rirpc Sfedu](http://rirpc.ru)

## [Instruction](https://github.com/GigaFlopsis/rc_car_ros/wiki)<br/>
![image](https://ibb.co/cggQKn][img]https://preview.ibb.co/mCggs7/car.png)<br/>

[![IMAGE ALT TEXT](http://img.youtube.com/vi/0fmgQAftFPY/0.jpg)](http://www.youtube.com/watch?v=0fmgQAftFPY "ROS Autonomous Navigation stack on RC car")

## Configuration:
In current version, the platform has a lidar [rplidar A2](https://www.slamtec.com/en/Lidar/A2#) and gyro [MPU6050](https://playground.arduino.cc/Main/MPU-6050) (optional).

## Complite tasks of the project:
* remote controll via ros_node
* [ros_node for remote controll with joystick](https://github.com/turtlebot/turtlebot/tree/kinetic/turtlebot_teleop)
* [hector_mapping](http://wiki.ros.org/hector_mapping) SLAM
* added Imu data from gyroscope MPU6050
* add [AMCL](http://wiki.ros.org/amcl)
* add Imu data from flight controller (cc3d via mavros)

## The current tasks of the project:
* write a controller
* add odometry using optical encoders (optional)
