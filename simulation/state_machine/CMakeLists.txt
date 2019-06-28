cmake_minimum_required(VERSION 2.8.3)
project(state_machine)

set(CMAKE_C_COMPILER   "gcc-7")
set(CMAKE_CXX_COMPILER "g++-7")

add_compile_options(--std=c++1z)

#[[find_package(DJIOSDK)
include_directories(${DJIOSDK_INCLUDE_DIR})]]

if(DJIOSDK_FOUND)
    add_definitions(-DHAVE_DJIOSDK)
    find_package(catkin REQUIRED
          roscpp
          std_msgs
          geometry_msgs
          nav_msgs
          sensor_msgs
          dji_sdk
          actionlib
          actionlib_msgs
          message_runtime
          quadrotor_msgs
          tf
          cmake_utils
        )
    message("${Green} state_machine package will be complied with DJI SDK support${ColourReset}")
else(DJIOSDK_FOUND)
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
    message("${Red} state_machine package cannot find DJI SDK. It would be compiled without DJI devices support${ColourReset}")
endif(DJIOSDK_FOUND)

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

enable_testing()

set(DRMEMORY $ENV{DRMEMORY})

macro (CMAKE_ADD_TEST NAME)
   add_executable(${NAME}_test test/${NAME}_test.cpp)
   target_link_libraries(${NAME}_test gtest pthread ${PROJECT_NAME})
   if("${DRMEMORY}" STREQUAL "")
       add_test(NAME ${NAME}_test
                COMMAND ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}_test)
   else()
       add_test(NAME ${NAME}_test
                COMMAND ${DRMEMORY} -brief -results_to_stderr -exit_code_if_errors 2 -- ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${NAME}_test)
   endif()
endmacro (CMAKE_ADD_TEST)

CMAKE_ADD_TEST(eventspinner)
CMAKE_ADD_TEST(statemachine)
CMAKE_ADD_TEST(mpaux)
CMAKE_ADD_TEST(mission)
CMAKE_ADD_TEST(dataio)
