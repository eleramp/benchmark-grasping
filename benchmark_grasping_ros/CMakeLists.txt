cmake_minimum_required(VERSION 2.8.3)
project(benchmark_grasping_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules eigen_conversions message_generation roscpp rospy tf2_ros sensor_msgs std_msgs geometry_msgs)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## System dependencies are found with CMake's conventions
set(INSTALL_DIR $ENV{INSTALL_DIR})
find_library(GPD_LIB NAMES gpd PATHS /usr/local/lib ${INSTALL_DIR} PATH_SUFFIXES lib NO_DEFAULT_PATH)
if (GPD_LIB)
  message(STATUS "Library GPD found in ${GPD_LIB}")
else()
  message(FATAL_ERROR "Library GPD not found")
endif()

set(GPD_LIB_INCLUDE_DIR $ENV{INSTALL_DIR}/include)
include_directories(${GPD_LIB_INCLUDE_DIR})
message(STATUS "gpd_include_dir: ${GPD_LIB_INCLUDE_DIR}")

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++14")

option(DEXNET_ROS "Compile dexnet ros" ON)
option(GPD_ROS "Compile gpd ros" ON)

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
  INCLUDE_DIRS gpd_ros/include
#  LIBRARIES benchmark_grasping_ros
#  DEPENDS system_lib
  CATKIN_DEPENDS cmake_modules message_runtime eigen_conversions roscpp rospy tf2_ros sensor_msgs std_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(gpd_ros/include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}_grasp_messages gpd_ros/src/gpd_ros/grasp_messages.cpp)
add_dependencies(${PROJECT_NAME}_grasp_messages ${catkin_EXPORTED_TARGETS})
add_library(${PROJECT_NAME}_grasp_plotter gpd_ros/src/gpd_ros/grasp_plotter.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_gpd_grasp_planner_server gpd_ros/src/gpd_ros/gpd_grasp_planner_server.cpp)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_gpd_grasp_planner_server PROPERTIES OUTPUT_NAME gpd_grasp_planner_server PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_gpd_grasp_planner_server
                      ${PROJECT_NAME}_grasp_messages
                      ${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${PCL_LIBRARIES}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_plotter
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_messages
                      ${GPD_LIB}
                      ${catkin_LIBRARIES})

