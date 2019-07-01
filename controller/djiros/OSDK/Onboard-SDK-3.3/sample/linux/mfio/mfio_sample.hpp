/*! @file mfio_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Multi-Function I/O API usage in a Linux environment.
 *  Shows example usage of the APIs available for controlling the MFIO pins
 *  on the vehicle/FC.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef DJIOSDK_MFIOSAMPLE_HPP
#define DJIOSDK_MFIOSAMPLE_HPP

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

bool pwmOutputBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool pwmOutputNonBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool gpioLoopbackBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool gpioLoopbackNonBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool adcBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);
bool adcNonBlockingApiDemo(DJI::OSDK::Vehicle* vehicle);

static void getGpiCallBack(DJI::OSDK::Vehicle*      vehicle,
                           DJI::OSDK::RecvContainer recvFrame,
                           DJI::OSDK::UserData      userData);
static void getAdcCallBack(DJI::OSDK::Vehicle*      vehicle,
                           DJI::OSDK::RecvContainer recvFrame,
                           DJI::OSDK::UserData      userData);

DJI::OSDK::UserData userData;

#endif // DJIOSDK_MFIOSAMPLE_HPP
