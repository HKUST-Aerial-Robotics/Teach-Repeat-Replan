cmake_minimum_required(VERSION 2.8.3)
project(djiros)

set(CMAKE_VERBOSE_MAKEFILE "true")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(ADDITIONAL_CXX_FLAG "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")


## Set this value to true if camera-imu sync function is needed
set(ENABLE_DJIFOX false)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
)

find_package(DJIOSDK REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Gimbal.msg
  Waypoint.msg
  MobileData.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Activation.srv
  CameraAction.srv
  DroneArmControl.srv
  DroneTaskControl.srv
  MFIOConfig.srv
  MFIOSetValue.srv
  MissionHpAction.srv
  MissionHpGetInfo.srv
  MissionHpResetYaw.srv
  MissionHpUpdateRadius.srv
  MissionHpUpdateYawRate.srv
  MissionHpUpload.srv
  MissionStatus.srv
  MissionWpAction.srv
  MissionWpGetInfo.srv
  MissionWpGetSpeed.srv
  MissionWpSetSpeed.srv
  MissionWpUpload.srv
  SDKControlAuthority.srv
  SendMobileData.srv
  SetHardSync.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
        message_runtime
        geometry_msgs
        nav_msgs
        std_msgs
)
 
###########
## Build ##  
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

file(GLOB DJIROS_SOURCES src/djiros/DjiRos*.cpp) 
file(GLOB DJI_OSDK_ROS_SOURCES src/modules/*.cpp) 

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

if(ENABLE_DJIFOX)
  find_package(mvIMPACT REQUIRED)
endif(ENABLE_DJIFOX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${mvIMPACT_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(dji_sdk
#   src/${PROJECT_NAME}/dji_sdk.cpp
# ) 

## Declare a cpp executable
add_executable(djiros_node 
  src/djiros/djiros_node.cpp
  ${DJIROS_SOURCES}
  ${DJI_OSDK_ROS_SOURCES}
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(djiros_node 
  djiros_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(djiros_node
   ${catkin_LIBRARIES}
   ${mvIMPACT_LIBRARIES}
   ${DJIOSDK_LIBRARIES}
)

if (ENABLE_DJIFOX)
  add_executable(djifox_node
    src/djiros/djifox_node.cpp
    src/bluefox2/camera.cpp
    ${DJIROS_SOURCES}
    ${DJI_OSDK_ROS_SOURCES}
  )

  target_link_libraries(djifox_node
     ${catkin_LIBRARIES}
     ${mvIMPACT_LIBRARIES}
     ${DJIOSDK_LIBRARIES}
  )

  add_dependencies(djifox_node 
    djiros_generate_messages_cpp)

endif(ENABLE_DJIFOX)

option(USE_COLLISION_AVOIDANCE "Use DJI collision avoidance library" OFF)
if(USE_COLLISION_AVOIDANCE)
  include(${CMAKE_MODULE_PATH}/External_CollisionAvoidance.cmake)
endif()

option(USE_POINTCLOUD2LAS "Use DJI Pointcloud to LAS converter library" OFF)
if(USE_POINTCLOUD2LAS)
  include(${CMAKE_MODULE_PATH}/External_Pointcloud2Las.cmake)
endif()
