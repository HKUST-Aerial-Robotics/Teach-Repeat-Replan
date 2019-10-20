cmake_minimum_required(VERSION 2.8.3)
project(polyhedron_generator)

find_package(catkin REQUIRED COMPONENTS 
    rospy 
    std_msgs 
    sensor_msgs 
    visualization_msgs 
    tf 
    quadrotor_msgs 
    decomp_ros_msgs
    cmake_utils
)

set(ENABLE_CUDA true)

if(ENABLE_CUDA)
    message("${Green} Use CUDA")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_61)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_61,code=sm_61)
    set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math )
    set(CUDA_PROPAGATE_HOST_FLAGS OFF)
    SET(CUDA_SEPARABLE_COMPILATION ON)
else(ENABLE_CUDA)
    message("${Red} Don't use CUDA")
endif(ENABLE_CUDA)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -march=native")

find_package(cmake_modules REQUIRED)
if(ENABLE_CUDA)
    find_package(CUDA REQUIRED )
endif(ENABLE_CUDA)
find_package(PCL REQUIRED)
find_package(Eigen3  REQUIRED)

set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

catkin_package(
    INCLUDE_DIRS include 
    LIBRARIES poly_convexify
    CATKIN_DEPENDS 
    rospy 
    std_msgs 
    sensor_msgs 
    visualization_msgs 
)

include_directories(
    include 
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

if(ENABLE_CUDA)
    include_directories(
        ${CUDA_INCLUDE_DIRS}
)
endif(ENABLE_CUDA)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(/usr/local/lib)

if(ENABLE_CUDA)
    cuda_add_library(poly_convexify STATIC
        src/cluster_server.cu
        src/cluster_engine.cu
        src/cluster_engine_cpu.cpp
    )
else(ENABLE_CUDA)
    add_library(poly_convexify STATIC
        src/cluster_server_cpu.cpp
        src/cluster_engine_cpu.cpp
    )
endif(ENABLE_CUDA)