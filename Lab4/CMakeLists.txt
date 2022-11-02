cmake_minimum_required(VERSION 2.8.3)
project(rbe3002_final)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
)

find_package(catkin REQUIRED
  turtlebot3_description
  turtlebot3_bringup
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES RBE3002_template
   CATKIN_DEPENDS rospy turtlebot3_bringup turtlebot3_description
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)