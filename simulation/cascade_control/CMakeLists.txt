cmake_minimum_required(VERSION 2.8.3)
project(cascade_controller)

add_compile_options(-std=c++11 -fPIC)

#[[find_package(DJIOSDK)
include_directories(${DJIOSDK_INCLUDE_DIR})]]

if(DJIOSDK_FOUND)
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      quadrotor_msgs
      geometry_msgs
      nav_msgs
      sensor_msgs
      dji_sdk
      nodelet
      uav_utils
      dynamic_reconfigure
      cmake_utils
    )
else(DJIOSDK_FOUND)
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
endif(DJIOSDK_FOUND)

find_package(GSL REQUIRED)
include_directories(${GSL_INCLUDE_DIR})
find_package(Eigen3  REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


generate_dynamic_reconfigure_options(
  cfg/params.cfg
)


if(DJIOSDK_FOUND)
    message("${Green} Cascade Control package will be complied with DJI SDK support${ColourReset}")
    add_definitions(-DHAVE_DJIOSDK)
    catkin_package(
      INCLUDE_DIRS include
    #  LIBRARIES cascade_controller
      CATKIN_DEPENDS roscpp std_msgs quadrotor_msgs  geometry_msgs nav_msgs dji_sdk uav_utils
      DEPENDS Eigen3 DJIOSDK Kalman
    )

    include_directories(
      include
      test
      ${catkin_INCLUDE_DIRS}
      ${Eigen3_INCLUDE_DIRS}
      ${DJIOSDK_INCLUDE_DIRS}
      ${GSL_INCLUDE_DIRS}
    )

    file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

    add_library(${PROJECT_NAME}
       ${${PROJECT_NAME}_SRCS}
     )
    add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        ${GSL_LIBRARIES}
        )
else(DJIOSDK_FOUND)
    message("${Red} Cascade Control package cannot find DJI SDK. It would be compiled without DJI devices support${ColourReset}")
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
      ${GSL_INCLUDE_DIRS}
    )

    file(GLOB ${PROJECT_NAME}_SRCS src/*.cpp)

    add_library(${PROJECT_NAME}
       ${${PROJECT_NAME}_SRCS}
     )
    add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${GSL_LIBRARIES}
        )
endif(DJIOSDK_FOUND)

enable_testing()

macro (CMAKE_ADD_TEST NAME)
   add_executable(${NAME}_test test/${NAME}_test.cpp)
   target_link_libraries(${NAME}_test gtest ${GSL_LIBRARIES} ${PROJECT_NAME})
   add_test(NAME ${NAME}_test
            COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}_test)
endmacro (CMAKE_ADD_TEST)

CMAKE_ADD_TEST(so3)
CMAKE_ADD_TEST(translation_control)
CMAKE_ADD_TEST(cascade_control)
CMAKE_ADD_TEST(gsl)

