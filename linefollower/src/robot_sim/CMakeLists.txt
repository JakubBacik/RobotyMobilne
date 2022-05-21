cmake_minimum_required(VERSION 3.0.2)
project(robot_sim)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp  
    gazebo_ros
    gazebo_ros_control
    gazebo_msgs
    geometry_msgs
    controller_manager
    joint_state_controller
    robot_state_publisher
)

catkin_package()

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

