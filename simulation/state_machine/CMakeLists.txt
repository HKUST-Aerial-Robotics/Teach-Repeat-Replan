cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

set(CMAKE_C_COMPILER   "gcc-7")
set(CMAKE_CXX_COMPILER "g++-7")

add_compile_options(--std=c++1z)

find_package(catkin REQUIRED
      roscpp
      std_msgs
      geometry_msgs
      nav_msgs
      sensor_msgs
      actionlib
      actionlib_msgs
      message_runtime
      quadrotor_msgs
      tf
      cmake_utils
    )

find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES state_machine
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Eigen3_INCLUDE_DIRS}
)

file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

add_library(${PROJECT_NAME}
   ${${PROJECT_NAME}_SRCS}
 )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node ${${PROJECT_NAME}_SRCS} node/statemachine_node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   )