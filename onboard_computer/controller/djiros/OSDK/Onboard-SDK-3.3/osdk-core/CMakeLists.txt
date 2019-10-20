cmake_minimum_required(VERSION 2.8)
project(djiosdk-core)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING
            "Choose the type of build Debug/Release/Coverage:"
            FORCE)
endif ()

# Add module path for <name>.cmake conf files
set(CURRENT_CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

## Set compiler flags accordingly to the build type
if (MSVC)
  include_directories(${Qt5Widgets_INCLUDES})
  add_definitions(${Qt5Widgets_DEFINITIONS})
  set(COMMON_CXX_FLAGS "-std=c++11 ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
endif (MSVC)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  set(COMMON_CXX_FLAGS "-std=c++11 -pthread")
endif ()

if (CMAKE_SYSTEM_NAME MATCHES Darwin)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread -g -O0")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
endif ()

if (COVERAGE)
  SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage -fPIC")
  SET(GCC_COVERAGE_LINK_FLAGS    "-lgcov")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${GCC_COVERAGE_COMPILE_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")
endif ()

if (CMAKE_BUILD_TYPE MATCHES "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "${COMMON_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
endif ()


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  api/inc
  protocol/inc
  hal/inc
  utility/inc
  platform/default/inc
)

# Use this if more platform is supported
if (CMAKE_SYSTEM_NAME MATCHES Linux)
  include_directories(
    platform/linux/inc
  )
endif()


## Declare a C++ library
FILE(GLOB OSDK_LIB_SRCS
  api/inc/*.hpp
  api/src/*.cpp
  protocol/src/*.cpp
  hal/src/*.cpp
  utility/src/*.cpp
  platform/default/src/*.cpp
  )

# Use this if more platform is supported
if (CMAKE_SYSTEM_NAME MATCHES Linux)
  FILE(GLOB OSDK_LIB_SRCS ${OSDK_LIB_SRCS} platform/linux/src/*.cpp)
endif()

add_library(${PROJECT_NAME}
  STATIC
  ${OSDK_LIB_SRCS})

target_link_libraries(${PROJECT_NAME} pthread)

################
# Installation #
################

set(INSTALL_LIB_DIR lib)
set(INSTALL_BIN_DIR bin)
set(INSTALL_INCLUDE_DIR include/djiosdk)
set(DEF_INSTALL_CMAKE_DIR lib/cmake/djiosdk)
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR})
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Add all targets to the build-tree export set
export(TARGETS djiosdk-core
  FILE "${PROJECT_BINARY_DIR}/djiosdkTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE djiosdk)

# Create the DJIOSDKConfig.cmake and DJIOSDKConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake.in
  "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake" @ONLY)

# ... for the install tree
set(CONF_INCLUDE_DIRS "\${DJIOSDK_CMAKE_DIR}/${REL_INCLUDE_DIR}")

configure_file(${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake.in
  "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake" @ONLY)

# ... for both
configure_file(${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfigVersion.cmake.in
  "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfigVersion.cmake" @ONLY)

# Install the DJIOSDKConfig.cmake and DJIOSDKConfigVersion.cmake
install(FILES
  "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfig.cmake"
  "${CURRENT_CMAKE_MODULE_PATH}/DJIOSDKConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT djiosdkTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)

FILE(GLOB OSDK_LIB_HEADERS
  api/inc/*.h*
  protocol/inc/*.h*
  hal/inc/*.h*
  utility/inc/*.h*
  default/inc/*.h*
  platform/linux/inc/*.h*
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  PUBLIC_HEADER "${OSDK_LIB_HEADERS}")

if(NOT INSTALL_LIB_DIR)
  message(STATUS, "Install to build folder" )
  set(INSTALL_LIB_DIR "${CMAKE_CURRENT_BINARY_DIR}/lib")
  set(INSTALL_INCLUDE_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
endif()

install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the osdk-core library to the "export-set"
  EXPORT djiosdkTargets
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}" COMPONENT dev)
