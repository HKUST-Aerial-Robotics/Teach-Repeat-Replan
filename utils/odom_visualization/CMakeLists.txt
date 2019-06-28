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


###########
## Build ##
###########
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(irobot_msgs
#   src/${PROJECT_NAME}/irobot_msgs.cpp
# )

## Declare a cpp executable
add_executable(odom_visualization src/odom_visualization.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(multi_map_visualization multi_map_server_messages_cpp)

## Specify libraries to link a library or executable target against
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   ##pose_utils
)


