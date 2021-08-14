cmake_minimum_required(VERSION 2.8)
project(traj_server)

# set(CMAKE_VERBOSE_MAKEFILE "false")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    quadrotor_msgs
    tf)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS})

link_directories(
    ${PCL_LIBRARY_DIRS})

add_executable (traj_server_node 
    src/traj_server.cpp)

target_link_libraries(traj_server_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES})

