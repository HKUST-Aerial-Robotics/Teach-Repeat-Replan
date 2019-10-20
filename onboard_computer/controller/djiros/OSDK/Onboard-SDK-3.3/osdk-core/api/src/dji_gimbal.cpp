/** @file dji_gimbal.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Gimbal API for OSDK library
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_gimbal.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

DJI::OSDK::Gimbal::Gimbal(Vehicle* vehicle)
  : vehicle(vehicle)
{
}

DJI::OSDK::Gimbal::~Gimbal()
{
}

void
DJI::OSDK::Gimbal::setAngle(Gimbal::AngleData* data)
{
  vehicle->protocolLayer->send(0, encrypt,
                               OpenProtocol::CMDSet::Control::gimbalAngle,
                               (unsigned char*)data, sizeof(Gimbal::AngleData));
}

void
DJI::OSDK::Gimbal::setSpeed(Gimbal::SpeedData* data)
{
  data->reserved = 0x80;
  vehicle->protocolLayer->send(0, encrypt,
                               OpenProtocol::CMDSet::Control::gimbalSpeed,
                               (unsigned char*)data, sizeof(Gimbal::SpeedData));
}
