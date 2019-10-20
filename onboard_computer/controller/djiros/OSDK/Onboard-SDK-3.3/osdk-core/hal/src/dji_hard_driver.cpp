/*! @file dji_hard_driver.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Serial device driver abstraction. Provided as an abstract class. Please
 *  inherit and implement for individual platforms.
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#include "dji_hard_driver.hpp"

using namespace DJI::OSDK;

//! @todo change to dji_logging method
char DJI::OSDK::buffer[DJI::OSDK::HardDriver::bufsize];

HardDriver::HardDriver()
{
}

HardDriver::~HardDriver()
{
}

/*bool
HardDriver::getDeviceStatus()
{
  return true;
}*/

void
HardDriver::displayLog(const char* buf)
{
  if (buf)
    DDEBUG("%s", buf);
  else
    DDEBUG("%s", DJI::OSDK::buffer);
}

MMU*
HardDriver::getMmu()
{
  return &mmu;
}
