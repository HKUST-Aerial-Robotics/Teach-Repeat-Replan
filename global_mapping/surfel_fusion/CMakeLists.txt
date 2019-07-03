cmake_minimum_required(VERSION 2.8.3)
project(surfel_fusion)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -DEIGEN_DONT_PARALLELIZE -pthread ${OpenMP_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    cv_bridge
    sensor_msgs
    nav_msgs
    message_filters
    pcl_ros
)

find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_executable(surfel_fusion
    src/ros_node.cpp
    src/surfel_map.cpp
    src/fusion_functions.cpp 
)

target_link_libraries(surfel_fusion 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${Boost_LIBRARIES}
)
