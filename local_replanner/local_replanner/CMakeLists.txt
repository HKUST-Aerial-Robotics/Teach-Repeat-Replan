cmake_minimum_required(VERSION 2.8.3)
project(trr_local_replanner)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    nav_msgs 
    visualization_msgs 
    quadrotor_msgs 
    tf  
    cv_bridge
    local_map
    cmake_utils
)

# find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED )
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS local_map
)

include_directories( 
    SYSTEM 
    include 
    third_party
    ${PROJECT_SOURCE_DIR}/third_party/mosek/include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

message(STATUS "${catkin_INCLUDE_DIRS}")
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable ( time_optimal_traj_server 
    src/traj_server.cpp 
    src/bezier_base.cpp
    src/non_uniform_bspline.cpp )
target_link_libraries( time_optimal_traj_server
    ${catkin_LIBRARIES} )

add_executable ( local_replanner 
    src/replanner_node.cpp 
    src/bspline_replanner.cpp
    src/grad_band_optimizer.cpp
    src/non_uniform_bspline.cpp 
    src/bezier_base.cpp
    )
target_link_libraries( local_replanner
    ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${NLOPT_LIBRARIES} )  