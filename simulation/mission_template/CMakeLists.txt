cmake_minimum_required(VERSION 2.8.3)
project(mission_template)

set(CMAKE_C_COMPILER   "gcc-7")
set(CMAKE_CXX_COMPILER "g++-7")
add_compile_options(-std=c++14 -Wignored-attributes  ${CMAKE_CXX_FLAGS})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  nodelet
  uav_utils
  state_machine
  pluginlib
  cmake_utils
)

find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mission_template
  CATKIN_DEPENDS roscpp pluginlib std_msgs quadrotor_msgs  geometry_msgs nav_msgs nodelet uav_utils state_machine
  DEPENDS Eigen3 
)

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

## Declare a C++ library
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_library(${PROJECT_NAME}
   ${${PROJECT_NAME}_SRCS}
 )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)