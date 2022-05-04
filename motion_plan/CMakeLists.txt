cmake_minimum_required(VERSION 3.0.2)
project(motion_plan)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  rospy
  roscpp  
  gazebo_ros
  gazebo_ros_control
  gazebo_msgs
  geometry_msgs
)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
