cmake_minimum_required(VERSION 2.6)
 
PROJECT(PureQT)
 
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
 
QT4_WRAP_UI(UISrcs MyWidget.ui)
QT4_WRAP_CPP(MOCSrcs MyWidget.h)
 
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR})
 
ADD_EXECUTABLE(TestProject main.cpp MyWidget.cpp ${MOCSrcs} ${UISrcs})
TARGET_LINK_LIBRARIES(TestProject ${QT_QTMAIN_LIBRARY} ${QT_LIBRARIES})
 
IF(WIN32) # Check if we are on Windows
  if(MSVC) # Check if we are using the Visual Studio compiler
    set_target_properties(TestProject PROPERTIES LINK_FLAGS_RELEASE "/SUBSYSTEM:WINDOWS")
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    # SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mwindows") # Not tested
  else()
    message(SEND_ERROR "You are using an unsupported Windows compiler! (Not MSVC or GCC)")
  endif()
elseif(UNIX)
  # Nothing special required
else()
  message(SEND_ERROR "You are on an unsupported platform! (Not Win32 or Unix)")
ENDIF()
