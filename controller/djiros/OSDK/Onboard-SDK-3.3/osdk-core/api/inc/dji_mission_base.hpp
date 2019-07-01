/** @file dji_mission_base.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission-Base abstract class for DJI OSDK library
 *  @details This is a low-level abstraction for having
 *  commonality between all missions.
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef ONBOARDSDK_DJI_MISSIONBASE_H
#define ONBOARDSDK_DJI_MISSIONBASE_H

#include "dji_command.hpp"
#include "dji_type.hpp"
#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

typedef enum MissionType {
  MISSION_MODE_A,
  MISSION_WAYPOINT,
  MISSION_HOTPOINT,
  MISSION_FOLLOW,
  MISSION_IOC
} MissionType;

class Vehicle;

/*! @brief Mission Base class for commonality between SDK Missions.
 *
 * @details You can inherit from this class if making a custom mission.
 */
class MissionBase
{
public:
  MissionBase(Vehicle* vehicle = 0)
    : vehicle(vehicle)
  {
  }
  virtual ~MissionBase()
  {
  }

  virtual void start(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode start(int timer) = 0;

  virtual void stop(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode stop(int timer) = 0;

  virtual void pause(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode pause(int timer) = 0;

  virtual void resume(VehicleCallBack callback = 0, UserData userData = 0) = 0;
  virtual ACK::ErrorCode resume(int timer) = 0;

protected:
  Vehicle* vehicle;
}; // class MissionBase

} // OSDK
} // DJI

#endif // ONBOARDSDK_DJI_MISSIONBASE_H
