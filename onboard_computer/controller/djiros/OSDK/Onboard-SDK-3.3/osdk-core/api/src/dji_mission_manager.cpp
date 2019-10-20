/** @file dji_mission_manager.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission Manager API for DJI OSDK library
 *  @details This is a high-level abstraction for handling/chaining missions
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */
#include "dji_mission_manager.hpp"

using namespace DJI;
using namespace DJI::OSDK;

MissionManager::MissionManager(Vehicle* vehiclePtr)
  : vehicle(vehiclePtr)
  , wpMission(NULL)
  , wayptCounter(0)
  , hotptCounter(0)
{
}

MissionManager::~MissionManager()
{
  for (int i = 0; i < wayptCounter; ++i)
  {
    delete wpMissionArray[i];
  }

  for (int i = 0; i < hotptCounter; ++i)
  {
    delete hpMissionArray[i];
  }
}

ACK::ErrorCode
MissionManager::init(DJI_MISSION_TYPE type, int timeout, UserData missionData)
{
  if (type == WAYPOINT)
  {
    return this->initWayptMission(timeout, missionData);
  }
  else if (type == HOTPOINT)
  {
    return this->initHotptMission(timeout, missionData);
  }
  else
  {
    DERROR("Cannot recognize the mission type provided\n");
    // @todo return a false ack
    ACK::ErrorCode ack;
    ack.data = false;
    return ack;
  }
}

void
MissionManager::init(DJI_MISSION_TYPE type, VehicleCallBack callback,
                     UserData missionData)
{
  if (type == WAYPOINT)
  {
    this->initWayptMission(callback, missionData);
  }
  else if (type == HOTPOINT)
  {
    // @note hotpoint init() doesn't have blocking calls, timeout use 10
    this->initHotptMission(10, missionData);
  }
  else
  {
    DERROR("Cannot recognize the mission type provided\n");
  }
}

ACK::ErrorCode
MissionManager::initWayptMission(int timeout, UserData wayptData)
{

  wpMissionArray[wayptCounter] = new WaypointMission(this->vehicle);
  wpMission                    = wpMissionArray[wayptCounter];
  wayptCounter++;

  // @todo timeout needs to be defined somewhere globally
  return wpMission->init((WayPointInitSettings*)wayptData, timeout);
}

void
MissionManager::initWayptMission(VehicleCallBack callback, UserData wayptData)
{
  wpMissionArray[wayptCounter] = new WaypointMission(this->vehicle);
  wpMission                    = wpMissionArray[wayptCounter];
  wayptCounter++;

  wpMission->init((WayPointInitSettings*)wayptData, callback, wayptData);
}

ACK::ErrorCode
MissionManager::initHotptMission(int timeout, UserData hotptData)
{

  hpMissionArray[hotptCounter] = new HotpointMission(this->vehicle);
  hpMission                    = hpMissionArray[hotptCounter];
  hotptCounter++;

  if (hotptData)
  {
    hpMission->setData((HotPointSettings*)hotptData);
  }
  else
  {
    hpMission->initData();
  }

  // @todo this initData() does not return ack
  ACK::ErrorCode ack;
  ack.data = ACK::SUCCESS;

  return ack;
}

void
MissionManager::initHotptMission(VehicleCallBack callback, UserData wayptData)
{
  /*! @note hotpoint init() doesn't have blocking calls
   *        put it here for future uses
   */
}

void
MissionManager::missionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                UserData userData)
{
  char           func[50];
  ACK::ErrorCode ack;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= sizeof(ACK::ErrorCode))
  {
    ack.info = recvFrame.recvInfo;
    ack.data = recvFrame.recvData.missionACK;

    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }
  }
  else
  {
    DERROR("ACK is exception,sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}

WaypointMission*
MissionManager::getWaypt(int index)
{
  if (index >= wayptCounter)
  {
    DERROR("The waypt index does not exist in Mission Manager\n");
    return NULL;
  }

  return wpMissionArray[index];
}

HotpointMission*
MissionManager::getHotpt(int index)
{
  if (index >= hotptCounter)
  {
    DERROR("The hotpt index does not exist in Mission Manager\n");
    return NULL;
  }

  return hpMissionArray[index];
}

void
MissionManager::printInfo()
{
  DSTATUS("Mission Manager status: \n");
  DSTATUS("There are %d waypt missions and %d hotpoint missions\n",
          wayptCounter, hotptCounter);
}
