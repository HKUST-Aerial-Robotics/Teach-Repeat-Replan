/*! @file dji_version.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Drone/SDK Version definition for DJI onboardSDK library
 *
 *  @note Since OSDK 3.2.2 (Feb 2017), versioning is handled by the SDK.
 *  You can use the Version::FW macro to target your code towards specific
 * platforms/firmware.
 *
 *  @copyright
 *  Copyright 2016-17 DJI. All rights reserved.
 * */

#include "dji_version.hpp"
#include "dji_open_protocol.hpp"

using namespace DJI;
using namespace DJI::OSDK;

const Version::FirmWare
Version::FW(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
  return (((a << 24) & 0xff000000) | ((b << 16) & 0x00ff0000) |
          ((c << 8) & 0x0000ff00) | (d & 0x000000ff));
}
