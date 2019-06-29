cmake_minimum_required(VERSION 2.8.3)
project(cascade_controller)

add_compile_options(-std=c++11 -fPIC)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quadrotor_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  nodelet
  uav_utils
  dynamic_reconfigure
  cmake_utils
)

find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


generate_dynamic_reconfigure_options(
  cfg/params.cfg
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES cascade_controller
  CATKIN_DEPENDS roscpp std_msgs quadrotor_msgs  geometry_msgs nav_msgs uav_utils
  DEPENDS Eigen3 DJIOSDK Kalman
)

include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_library(${PROJECT_NAME}
   ${${PROJECT_NAME}_SRCS}
 )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )