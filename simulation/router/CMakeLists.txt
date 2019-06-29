cmake_minimum_required(VERSION 2.8.3)
project(router)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_C_COMPILER   "gcc-7")
set(CMAKE_CXX_COMPILER "g++-7")
add_compile_options(-std=c++14 -Wignored-attributes  ${CMAKE_CXX_FLAGS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  quadrotor_msgs
  geometry_msgs
  mission_template
  nav_msgs
  sensor_msgs
  nodelet
  uav_utils
  state_machine
  pluginlib
  cmake_utils
)

## System dependencies are found with CMake's conventions
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# find_package(Kalman REQUIRED)
# include_directories(${Kalman_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES mission_template
  CATKIN_DEPENDS roscpp nav_msgs pluginlib std_msgs quadrotor_msgs mission_template geometry_msgs nodelet uav_utils state_machine
  DEPENDS Eigen3 Kalman mission_template
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  test
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  # ${Kalman_INCLUDE_DIRS}
)

## Declare a C++ library
file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_library(${PROJECT_NAME}
   ${${PROJECT_NAME}_SRCS}
 )

set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/../../../../../devel/lib/libmission_template.so)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    #mission_template
    )

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)


enable_testing()

macro (CMAKE_ADD_TEST NAME)
   add_executable(${NAME}_test test/${NAME}_test.cpp)
   target_link_libraries(${NAME}_test gtest ${PROJECT_NAME})
   add_test(NAME ${NAME}_test
            COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}_test)
 endmacro (CMAKE_ADD_TEST)
