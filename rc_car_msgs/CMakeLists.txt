cmake_minimum_required(VERSION 2.8.3)
project(rc_car_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)


add_message_files(FILES 
  Diagnostics.msg
  CarParams.msg
  CarPwmContol.msg
  CarPose.msg
)

#add_service_files(
#   FILES
#    msg.srv
#)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
   geometry_msgs
   nav_msgs
   sensor_msgs
   std_msgs
)

## catkin specific configuration 
catkin_package(
   CATKIN_DEPENDS message_runtime
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)


