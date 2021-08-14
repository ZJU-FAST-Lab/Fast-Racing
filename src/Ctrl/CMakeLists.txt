cmake_minimum_required(VERSION 2.8.3)
project(Ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  quadrotor_msgs
  geometry_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# add_message_files(
#   FILES
#   PositionCommand.msg
# )

# generate_messages(
#     DEPENDENCIES
#     geometry_msgs
#     std_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Ctrl
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  include/${PROJECT_NAME}
  include
)

## Declare a cpp executable
add_executable(Ctrl_node 
	src/Ctrl_node.cpp
  src/CtrlFSM.cpp
	src/CtrlParam.cpp
  src/controller.cpp
  src/hovthrkf.cpp
  src/input.cpp
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(Ctrl_node quadrotor_msgs)

## Specify libraries to link a library or executable target against
target_link_libraries(Ctrl_node
  ${catkin_LIBRARIES}
)

