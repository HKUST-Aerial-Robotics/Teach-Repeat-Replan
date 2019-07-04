cmake_minimum_required(VERSION 2.8.3)
project(map_server)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs
    nav_msgs
    visualization_msgs 
    tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
#  INCLUDE_DIRS include
)

include_directories(
#    include 
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable ( map_server src/map_server.cpp)
target_link_libraries( map_server
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES}
)