#-------------------------------------------------
#
# Project created by QtCreator 2015-11-23T16:26:43
#
#-------------------------------------------------

TARGET = djiosdk-qt-sample
TEMPLATE = app
CONFIG += c++11

QT       += core gui
CONFIG   += console
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += serialport

#-------------------------------------------------
#
# Project management
#
#-------------------------------------------------


HEADERS  += \
    qtosdk.hpp


SOURCES += main.cpp\
    qtosdk.cpp \

DJILIB += ONBOARDSDK\
          DJILIBCORE

DEPENDENCE += QT\
              DJILIB\
              DEPENDENCE

DEFINES += $$DEPENDENCE


message("DJILIB:"$$DJILIB)

contains(DJILIB,ONBOARDSDK){
QT += serialport
ONBOARDSDK_SRC += \
    $$PWD/../../../osdk-core/api/src/*.cpp \
    $$PWD/../../../osdk-core/hal/src/*.cpp \
    $$PWD/../../../osdk-core/platform/default/src/*.cpp \
    $$PWD/../../../osdk-core/platform/qt/src/*.cpp \
    $$PWD/../../../osdk-core/protocol/src/*.cpp \
    $$PWD/../../../osdk-core/utility/src/*.cpp \

HEADERS  += \
    $$PWD/../../../osdk-core/api/inc/*.hpp \
    $$PWD/../../../osdk-core/hal/inc/*.hpp \
    $$PWD/../../../osdk-core/platform/default/inc/*.hpp \
    $$PWD/../../../osdk-core/platform/qt/inc/*.hpp \
    $$PWD/../../../osdk-core/protocol/inc/*.hpp \
    $$PWD/../../../osdk-core/utility/inc/*.hpp \

SOURCES += $$ONBOARDSDK_SRC

HEADERS += $$ONBOARDSDK_INC

include(FlightControl/flight_control_panel.pri)
include(CameraGimbalControl/camera_gimbal_control_panel.pri)
include(Broadcast/broadcast.pri)
include(Subscribe/SubscribePanel.pri)
include(Missions/Missions.pri)
include(MFIO/mfio_panel.pri)

}

FORMS    +=     qtosdk.ui


INCLUDEPATH += \
            $$PWD/../../../osdk-core/api/inc \
            $$PWD/../../../osdk-core/hal/inc \
            $$PWD/../../../osdk-core/utility/inc \
            $$PWD/../../../osdk-core/protocol/inc \
            $$PWD/../../../osdk-core/platform/qt/inc \
            $$PWD/../../../osdk-core/platform/default/inc \

RESOURCES +=\
            resources.qrc

DISTFILES += \
    CMakeLists.txt
    FlightControl/flight_control_panel.pri


message("finish compile")
