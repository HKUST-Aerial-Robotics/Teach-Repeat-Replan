cmake_minimum_required(VERSION 2.8.3)
project(local_map)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  cv_bridge
  message_filters
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES local_map
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    third_party
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_library( local_map 
    src/sdf_map.cpp 
    src/raycast.cpp 
    )
target_link_libraries( local_map
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    )  
