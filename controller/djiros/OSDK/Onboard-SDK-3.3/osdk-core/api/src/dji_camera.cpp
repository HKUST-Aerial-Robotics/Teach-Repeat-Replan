/** @file dji_camera.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Camera/Gimbal API for DJI onboardSDK library
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#include "dji_camera.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

Camera::Camera(Vehicle* vehicle)
  : vehicle(vehicle)
{
}

Camera::~Camera()
{
}

void
Camera::shootPhoto()
{
  action(OpenProtocol::CMDSet::Control::cameraShot);
}

void
Camera::videoStart()
{
  action(OpenProtocol::CMDSet::Control::cameraVideoStart);
}

void
Camera::videoStop()
{
  action(OpenProtocol::CMDSet::Control::cameraVideoStop);
}

void
Camera::action(const uint8_t cmd[])
{
  uint8_t sendData = 0;
  vehicle->protocolLayer->send(0, encrypt, cmd, &sendData, 1);
}
