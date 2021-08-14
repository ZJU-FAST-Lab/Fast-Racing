cmake_minimum_required(VERSION 3.10.0)
project(plan_manage)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(AIRSIM_ROOT ${PROJECT_SOURCE_DIR}/../thirdparty/)
set(ENABLE_CUDA false)



find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  octomap_ros
  octomap_msgs
  octomap_server
  path_searching
  decomp_ros_utils
  airsim_ros_pkgs
)
include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${DECOMP_UTIL_INCLUDE_DIRS}
    # ${AIRSIM_ROOT}/AirLib/deps/eigen3
    ${AIRSIM_ROOT}/AirLib/include
    # ${AIRSIM_ROOT}/MavLinkCom/include
    # ${AIRSIM_ROOT}/MavLinkCom/common_utils
)


if(ENABLE_CUDA)
  find_package(CUDA REQUIRED)
  catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_searching
  DEPENDS OpenCV Eigen Boost
  )
  SET(CUDA_SEPARABLE_COMPILATION ON)
  SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math;)
  set(CUDA_NVCC_FLAGS 
  #       -gencode arch=compute_20,code=sm_20;
  #       -gencode arch=compute_20,code=sm_21;
  #       -gencode arch=compute_30,code=sm_30;
  #       -gencode arch=compute_35,code=sm_35;
  #       -gencode arch=compute_50,code=sm_50;
  #       -gencode arch=compute_52,code=sm_52; 
  #       -gencode arch=compute_60,code=sm_60;
  #       -gencode arch=compute_61,code=sm_61;
        -gencode arch=compute_75,code=sm_75;
  ) 
  set(CUDA_SEPARABLE_COMPILATION ON) 



  link_directories( 
          /usr/local/cuda/lib64
  )
  CUDA_ADD_LIBRARY( cuda_computer
        src/cuda_computer.cu
        OPTIONS  -rdc=true 
        -Xcompiler '-fPIC'
    )
  target_link_libraries(cuda_computer cublas
  ${CUDA_LIBRARIES}
  cudadevrt)

  add_executable(se3_node
      src/se3_node_gpu.cpp
      src/se3_planner.cpp  
      src/MinCoPlan_GPU.cpp
  )
  target_link_libraries(se3_node
    cuda_computer 
    ${catkin_LIBRARIES}
    AirLib
  )
else(ENABLE_CUDA)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES path_searching
    DEPENDS OpenCV Eigen Boost
  )
  add_executable(se3_node
      src/se3_node_cpu.cpp
      src/se3_planner.cpp  
      src/MinCoPlan_CPU.cpp
  )
  target_link_libraries(se3_node
    ${catkin_LIBRARIES}
    AirLib
  )

endif(ENABLE_CUDA)




