/**@file dji_virtual_rc.cpp
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Virtual Radio Control API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include "dji_virtual_rc.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

VirtualRC::VirtualRC(Vehicle* vehicle)
{
  this->vehicle = vehicle;
  resetVRCData();
}

void
VirtualRC::setControl(bool enable, VirtualRC::CutOff cutoffType)
{
  VirtualRCSetting setting;
  setting.cutoff = cutoffType;
  setting.enable = enable ? 1 : 0;
  vehicle->protocolLayer->send(0, DJI::OSDK::encrypt,
                               OpenProtocol::CMDSet::VirtualRC::settings,
                               &setting, sizeof(setting));
}

void
VirtualRC::sendData(VirtualRCData data)
{
  vrcData = data;
  vehicle->protocolLayer->send(0, DJI::OSDK::encrypt,
                               OpenProtocol::CMDSet::VirtualRC::data, &vrcData,
                               sizeof(vrcData));
}

void
VirtualRC::resetVRCData()
{
  vrcData.roll       = 1024;
  vrcData.pitch      = 1024;
  vrcData.throttle   = 1024;
  vrcData.yaw        = 1024;
  vrcData.gear       = 1024;
  vrcData.reserved   = 1024;
  vrcData.mode       = 1024;
  vrcData.Channel_07 = 1024;
  vrcData.Channel_08 = 1024;
  vrcData.Channel_09 = 1024;
  vrcData.Channel_10 = 1024;
  vrcData.Channel_11 = 1024;
  vrcData.Channel_12 = 1024;
  vrcData.Channel_13 = 1024;
  vrcData.Channel_14 = 1024;
  vrcData.Channel_15 = 1024;
}

void
VirtualRC::neutralVRCSticks()
{
  resetVRCData();
  vehicle->protocolLayer->send(0, DJI::OSDK::encrypt,
                               OpenProtocol::CMDSet::VirtualRC::data, &vrcData,
                               sizeof(vrcData));
}

Telemetry::RC
VirtualRC::getRCData() const
{
  return vehicle->broadcast->getRC();
}

VirtualRCData
VirtualRC::getVRCData() const
{
  return vrcData;
}

void
VirtualRC::setVRCData(const VirtualRCData& value)
{
  vrcData = value;
}

bool
VirtualRC::isVirtualRC() const
{
  return vehicle->broadcast->getSDKInfo().vrcStatus == 0 ? false : true;
}

Telemetry::RC
VirtualRC::toRCData(VirtualRCData& vData)
{
  Telemetry::RC rcData;
  rcData.gear     = (vData.gear == 1324) ? -454 : -10000;
  rcData.mode     = (1024 - vData.mode) * 10000 / 660;
  rcData.pitch    = (vData.pitch - 1024) * (10000) / 660;
  rcData.roll     = (vData.roll - 1024) * (10000) / 660;
  rcData.yaw      = (vData.yaw - 1024) * (10000) / 660;
  rcData.throttle = (vData.throttle - 1024) * (10000) / 660;
  return rcData;
}

VirtualRCData
VirtualRC::toVirtualRCData(Telemetry::RC& rcData)
{
  VirtualRCData vd;
  vd.yaw        = rcData.yaw * 660 / 10000 + 1024;
  vd.throttle   = rcData.throttle * 660 / 10000 + 1024;
  vd.pitch      = rcData.pitch * 660 / 10000 + 1024;
  vd.roll       = rcData.roll * 660 / 10000 + 1024;
  vd.gear       = (rcData.gear == -4545) ? 1324 : 1684;
  vd.reserved   = 1024;
  vd.mode       = rcData.mode * 660 / (-10000) + 1024;
  vd.Channel_07 = 1024;
  vd.Channel_08 = 1024;
  vd.Channel_09 = 1024;
  vd.Channel_10 = 1024;
  vd.Channel_11 = 1024;
  vd.Channel_12 = 1024;
  vd.Channel_13 = 1024;
  vd.Channel_14 = 1024;
  vd.Channel_15 = 1024;
  return vd;
}

Vehicle*
VirtualRC::getVehicle() const
{
  return this->vehicle;
}

void
VirtualRC::setVehicle(Vehicle* v)
{
  this->vehicle = v;
}
