cmake_minimum_required(VERSION 2.8.3)
project(joy_ctrl)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  quadrotor_msgs
  sensor_msgs
  roscpp
  std_msgs
  tf
  uav_utils
)

find_package(Eigen3)


catkin_package(
  LIBRARIES joy_ctrl
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/joy_ctrl.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_INCLUDE_DIR})
add_dependencies(${PROJECT_NAME}  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
