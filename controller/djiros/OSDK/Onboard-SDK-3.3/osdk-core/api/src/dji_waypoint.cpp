/** @file dji_waypoint.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of GPS Waypoint Missions for DJI OSDK
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#include "dji_waypoint.hpp"
#include "dji_mission_manager.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

WaypointMission::WaypointMission(Vehicle* vehicle)
  : MissionBase(vehicle)
  , index(NULL)
{
  wayPointEventCallback.callback = 0;
  wayPointEventCallback.userData = 0;
  wayPointCallback.callback      = 0;
  wayPointCallback.userData      = 0;
}

WaypointMission::~WaypointMission()
{
}

void
WaypointMission::init(WayPointInitSettings* Info, VehicleCallBack callback,
                      UserData userData)
{
  if (Info)
    setInfo(*Info);

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&MissionManager::missionCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointInit,
                               &info, sizeof(info), 500, 2, true, cbIndex);
}

ACK::ErrorCode
WaypointMission::init(WayPointInitSettings* Info, int timeout)
{

  ACK::ErrorCode ack;

  if (Info)
  {
    setInfo(*Info);
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointInit,
                               &info, sizeof(info), 500, 2, false, 2);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointInit, timeout));

  return ack;
}

void
WaypointMission::start(VehicleCallBack callback, UserData userData)
{
  uint8_t start = 0;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&MissionManager::missionCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetStart,
                               &start, sizeof(start), 500, 2, true, cbIndex);
}

ACK::ErrorCode
WaypointMission::start(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        start = 0;

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetStart,
                               &start, sizeof(start), 500, 2, false, 2);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointSetStart, timeout));

  return ack;
}

void
WaypointMission::stop(VehicleCallBack callback, UserData userData)
{
  uint8_t stop = 1;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&MissionManager::missionCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetStart,
                               &stop, sizeof(stop), 500, 2, true, cbIndex);
}

ACK::ErrorCode
WaypointMission::stop(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        stop = 1;

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetStart,
                               &stop, sizeof(stop), 500, 2, false, 2);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointSetStart, timeout));

  return ack;
}

void
WaypointMission::pause(VehicleCallBack callback, UserData userData)
{
  uint8_t data = 0;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&MissionManager::missionCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetPause,
                               &data, sizeof(data), 500, 2, true, cbIndex);
}

ACK::ErrorCode
WaypointMission::pause(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        data = 0;

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetPause,
                               &data, sizeof(data), 500, 2, false, 2);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointSetPause, timeout));

  return ack;
}

void
WaypointMission::resume(VehicleCallBack callback, UserData userData)
{
  uint8_t data = 1;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&MissionManager::missionCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetPause,
                               &data, sizeof(data), 500, 2, true, cbIndex);
}

ACK::ErrorCode
WaypointMission::resume(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        data = 1;

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointSetPause,
                               &data, sizeof(data), 500, 2, false, 2);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointSetPause, timeout));

  return ack;
}

ACK::WayPointInit WaypointMission::getWaypointSettings(int timer)
{
  ACK::WayPointInit ack;
  uint8_t arbNumber = 0;

  vehicle->protocolLayer->send(2, encrypt, OpenProtocol::CMDSet::Mission::waypointDownload, &arbNumber, sizeof(arbNumber), 1000, 4, 0, 0);

  ack = *((ACK::WayPointInit*)vehicle->waitForACK(
      OpenProtocol::CMDSet::Mission::waypointDownload, timer));

  return ack;
}

void WaypointMission::getWaypointSettings(VehicleCallBack callback, UserData userData)
{
  uint8_t arbNumber = 0;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)getWaypointSettingsCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
    OpenProtocol::CMDSet::Mission::waypointDownload, &arbNumber, sizeof(arbNumber), 1000, 4,
    true, cbIndex);
}

void
WaypointMission::getWaypointSettingsCallback(Vehicle*      vehicle,
					     RecvContainer recvFrame,
					     UserData      userData)
{
  ACK::WayPointInit wpInitInfo;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <=
      sizeof(ACK::WayPointInit))
  {
    wpInitInfo.data     = recvFrame.recvData.wpInitACK.data;
    wpInitInfo.ack.data = recvFrame.recvData.wpInitACK.ack;
    wpInitInfo.ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ACK::getError(wpInitInfo.ack))
    ACK::getErrorCodeMessage(wpInitInfo.ack, __func__);

  vehicle->missionManager->wpMission->info = wpInitInfo.data;

  DSTATUS("Index number: %d\n",
          vehicle->missionManager->wpMission->info.indexNumber);
}

ACK::WayPointIndex WaypointMission::getIndex(uint8_t index, int timer)
{
  ACK::WayPointIndex ack;

  vehicle->protocolLayer->send(2, encrypt, OpenProtocol::CMDSet::Mission::waypointIndexDownload, &index, sizeof(index), 1000, 4, 0, 0);

  ack = *((ACK::WayPointIndex*)vehicle->waitForACK(
        OpenProtocol::CMDSet::Mission::waypointIndexDownload, timer));

  return ack;
}

void WaypointMission::getIndex(uint8_t index, VehicleCallBack callback, UserData userData)
{
  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)getIndexCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(2, encrypt,
    OpenProtocol::CMDSet::Mission::waypointIndexDownload,
    &index, sizeof(index), 1000, 4, true, cbIndex);
}

void
WaypointMission::getIndexCallback(Vehicle*      vehicle,
                                  RecvContainer recvFrame,
                                  UserData      userData)
{
  ACK::WayPointIndex wpIndexInfo;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <=
      sizeof(ACK::WayPointIndex))
  {
    wpIndexInfo.data     = recvFrame.recvData.wpIndexACK.data;
    wpIndexInfo.ack.data = recvFrame.recvData.wpIndexACK.ack;
    wpIndexInfo.ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ACK::getError(wpIndexInfo.ack))
    ACK::getErrorCodeMessage(wpIndexInfo.ack, __func__);

  vehicle->missionManager->wpMission->index = &(wpIndexInfo.data);

  DSTATUS("Index number: %d\n",
          vehicle->missionManager->wpMission->index->index);
}

/*WayPointInitSettings
WaypointMission::getInfo() const
{
  return info;
}*/

void
WaypointMission::setInfo(const WayPointInitSettings& value)
{
  //! @todo set information for way point
  info = value;
  for (int i         = 0; i < 16; ++i)
    info.reserved[i] = 0;

  // TODO this might affect something, don't delete it yet for future debug use
  //#ifndef STATIC_MEMORY
  //  if (index != 0) delete index;
  //  index = 0;
  //#else
  //  if (maxIndex < info.indexNumber) index = 0;
  //#endif  // STATIC_MEMORY
}

void
WaypointMission::setIndex(WayPointSettings* value, size_t pos)
{
  if (index == 0)
  {
    index = new WayPointSettings[info.indexNumber];
    if (index == NULL)
    {
      DERROR("Lack of memory\n");
      return;
    }
  }
  index[pos] = *value;
  for (int i               = 0; i < 8; ++i)
    index[pos].reserved[i] = 0;
}

/*WayPointSettings*
WaypointMission::getIndex() const
{
  return index;
}

WayPointSettings*
WaypointMission::getIndex(size_t pos) const
{
  return &(index[pos]);
}*/

bool
WaypointMission::uploadIndexData(WayPointSettings* data,
                                 VehicleCallBack callback, UserData userData)
{
  setIndex(data, data->index);

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&WaypointMission::uploadIndexDataCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  WayPointSettings send;
  if (data->index < info.indexNumber)
    send = index[data->index];
  else
    return false; //! @note range error

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointAddPoint,
                               &send, sizeof(send), 1000, 4, true, cbIndex);
}

ACK::WayPointIndex
WaypointMission::uploadIndexData(WayPointSettings* data, int timeout)
{
  WayPointSettings   wpData;
  ACK::WayPointIndex ack;

  setIndex(data, data->index);

  if (data->index < info.indexNumber)
  {
    wpData = index[data->index];
  }
  else
  {
    // TODO add error handling
    DERROR("Range error\n");
  }

  vehicle->protocolLayer->send(2, encrypt,
                               OpenProtocol::CMDSet::Mission::waypointAddPoint,
                               &wpData, sizeof(wpData), 1000, 4, false, 2);

  ack = *((ACK::WayPointIndex*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointAddPoint, timeout));

  return ack;
}

void
WaypointMission::readIdleVelocity(VehicleCallBack callback, UserData userData)
{
  uint8_t zero = 0;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&WaypointMission::idleVelocityCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(
    2, encrypt, OpenProtocol::CMDSet::Mission::waypointGetVelocity, &zero,
    sizeof(zero), 500, 2, true, cbIndex);
}

ACK::ErrorCode
WaypointMission::readIdleVelocity(int timeout)
{
  ACK::ErrorCode ack;
  uint8_t        zero = 0;

  vehicle->protocolLayer->send(
    2, encrypt, OpenProtocol::CMDSet::Mission::waypointGetVelocity, &zero,
    sizeof(zero), 500, 2, false, 0);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointGetVelocity, timeout));

  return ack;
}

void
WaypointMission::updateIdleVelocity(float32_t       meterPreSecond,
                                    VehicleCallBack callback, UserData userData)
{
  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] =
      (void*)&WaypointMission::idleVelocityCallback;
    vehicle->nbUserData[cbIndex] = NULL;
  }

  vehicle->protocolLayer->send(
    2, encrypt, OpenProtocol::CMDSet::Mission::waypointSetVelocity,
    &meterPreSecond, sizeof(meterPreSecond), 500, 2, true, cbIndex);
}

ACK::WayPointVelocity
WaypointMission::updateIdleVelocity(float32_t meterPreSecond, int timeout)
{
  ACK::WayPointVelocity ack;

  vehicle->protocolLayer->send(
    2, encrypt, OpenProtocol::CMDSet::Mission::waypointSetVelocity,
    &meterPreSecond, sizeof(meterPreSecond), 500, 2, false, 0);

  ack = *((ACK::WayPointVelocity*)vehicle->waitForACK(
    OpenProtocol::CMDSet::Mission::waypointSetVelocity, timeout));

  return ack;
}

void
WaypointMission::idleVelocityCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                      UserData userData)
{
  ACK::WayPointVelocityInternal wpIdleVelocityInfo;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <=
      sizeof(ACK::WayPointVelocityInternal))
  {
    wpIdleVelocityInfo = recvFrame.recvData.wpVelocityACK;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  ACK::ErrorCode ack;
  ack.data = wpIdleVelocityInfo.ack;
  ack.info = recvFrame.recvInfo;

  if (ACK::getError(ack))
    ACK::getErrorCodeMessage(ack, __func__);

  vehicle->missionManager->wpMission->info.idleVelocity =
    wpIdleVelocityInfo.idleVelocity;

  DSTATUS("Current idle velocity: %f\n",
          vehicle->missionManager->wpMission->info.idleVelocity);
}

void
WaypointMission::uploadIndexDataCallback(Vehicle*      vehicle,
                                         RecvContainer recvFrame,
                                         UserData      userData)
{
  ACK::WayPointAddPointInternal wpDataInfo;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <=
      sizeof(ACK::WayPointAddPointInternal))
  {
    wpDataInfo = recvFrame.recvData.wpAddPointACK;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  ACK::ErrorCode ack;
  ack.data = wpDataInfo.ack;
  ack.info = recvFrame.recvInfo;

  if (ACK::getError(ack))
    ACK::getErrorCodeMessage(ack, __func__);

  DSTATUS("Index number: %d\n", wpDataInfo.index);
}

void
WaypointMission::setWaypointEventCallback(VehicleCallBack callback,
                                          UserData        userData)
{
  wayPointEventCallback.callback = callback;
  wayPointEventCallback.userData = userData;
}

void
WaypointMission::setWaypointCallback(VehicleCallBack callback,
                                     UserData        userData)
{
  wayPointCallback.callback = callback;
  wayPointCallback.userData = userData;
}
