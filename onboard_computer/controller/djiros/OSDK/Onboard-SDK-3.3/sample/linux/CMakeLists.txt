cmake_minimum_required(VERSION 2.8)
project(onboardsdk-linux-samples)

if(NOT ONBOARDSDK_SOURCE)
    set(ONBOARDSDK_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../../osdk-core")
endif()

add_subdirectory(camera-gimbal)
add_subdirectory(flight-control)
add_subdirectory(mfio)
add_subdirectory(missions)
add_subdirectory(mobile)
add_subdirectory(telemetry)