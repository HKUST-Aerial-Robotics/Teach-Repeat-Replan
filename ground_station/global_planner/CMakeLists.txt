cmake_minimum_required(VERSION 2.8.3)
project(trr_global_planner)

set(ENABLE_CUDA false)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    std_msgs 
    nav_msgs 
    visualization_msgs 
    quadrotor_msgs 
    decomp_ros_msgs
    tf 
    polyhedron_generator 
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

if(ENABLE_CUDA)
    find_package(CUDA REQUIRED )
endif(ENABLE_CUDA)

set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
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

if(ENABLE_CUDA)
    include_directories( 
        ${CUDA_INCLUDE_DIRS}
    )
endif(ENABLE_CUDA)

link_directories(${PROJECT_SOURCE_DIR}/third_party/mosek/lib/mosek8_1)
link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall") 

add_library( poly_utils STATIC
    src/utils/poly_utils.cpp
    third_party/quickhull/QuickHull.cpp )

add_executable( tr_node 
    src/utils/a_star.cpp
    src/utils/bezier_base.cpp
    third_party/eigen-cdd/Polyhedron.cpp
    src/teach_repeat_planner.cpp
    src/spatial_optimizer.cpp
    src/temporal_optimizer.cpp )

target_link_libraries( tr_node 
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES} 
    mosek64 
    ooqpgensparse 
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran f2c 
    cdd glpk poly_utils poly_convexify )

add_executable ( random_complex 
    src/utils/random_complex_generator.cpp )

target_link_libraries( random_complex
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} )  

add_executable ( click_obstacle_generation 
    src/utils/click_obstacle_generation.cpp )

target_link_libraries( click_obstacle_generation
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} ) 