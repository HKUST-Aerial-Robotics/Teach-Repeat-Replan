cmake_minimum_required(VERSION 2.8.3)
project(odom_visualization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  tf
  pose_utils
  cmake_utils
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs std_msgs visualization_msgs tf pose_utils
  DEPENDS system_lib
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(odom_visualization src/odom_visualization.cpp)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES})