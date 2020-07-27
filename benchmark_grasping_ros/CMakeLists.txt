cmake_minimum_required(VERSION 2.8.3)
project(benchmark_grasping_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules message_generation roscpp rospy tf2_ros sensor_msgs std_msgs geometry_msgs)

option(DEXNET_ROS "Compile dexnet ros" OFF)
option(GPD_ROS "Compile gpd ros" OFF)

if(DEXNET_ROS)
    add_subdirectory(dexnet_ros)
endif()

if(GPD_ROS)
    add_subdirectory(gpd_ros)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BenchmarkGrasp.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  UserCmd.srv
  GraspPlanner.srv
  GraspPlannerCloud.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES benchmark_grasping_ros
#  DEPENDS system_lib
  CATKIN_DEPENDS cmake_modules message_runtime roscpp rospy tf2_ros sensor_msgs std_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

