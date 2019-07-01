/*! @file dji_linux_helpers.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Helper functions to handle user configuration parsing, version query and
 * activation.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef ONBOARDSDK_HELPERS_H
#define ONBOARDSDK_HELPERS_H

#include <dji_linux_environment.hpp>
#include <dji_vehicle.hpp>

DJI::OSDK::Vehicle* setupOSDK(int argc, char** argv);
bool validateSerialDevice(DJI::OSDK::LinuxSerialDevice* serialDevice);

#endif // ONBOARDSDK_HELPERS_H
