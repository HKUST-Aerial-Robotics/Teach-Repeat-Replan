/*! @file main.cpp
*  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  An exmaple program of DJI-onboard-SDK portable for stm32
 *
 *  Copyright 2016 DJI. All right reserved.
 *  */

#ifndef MAIN_H
#define MAIN_H

#include "Activate.h"
#include "BspUsart.h"
#include "CameraGimbalSample.h"
#include "FlightControlSample.h"
#include "MissionSample.h"
#include "MobileSample.h"
#include "Receive.h"
#include "TelemetrySample.h"
#include "bsp.h"
#include "cppforstm32.h"
#include "dji_vehicle.hpp"
#include "stm32f4xx_conf.h"
#include "timer.h"
extern uint32_t tick; // tick is the time stamp,which record how many ms since u
                      // initialize the system.
// warnning: after 49 days of non-reset running, tick will RESET to ZERO.

#endif // MAIN_H
