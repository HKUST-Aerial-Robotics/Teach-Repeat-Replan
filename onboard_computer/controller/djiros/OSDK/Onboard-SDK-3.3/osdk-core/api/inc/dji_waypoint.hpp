/** @file dji_waypoint.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of GPS Waypoint Missions for DJI OSDK
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#ifndef DJI_WAYPOINT_H
#define DJI_WAYPOINT_H

#include "dji_mission_base.hpp"

namespace DJI
{
namespace OSDK
{

/*! @brief APIs for GPS Waypoint Missions
 *
 *  @details This class inherits from MissionBase and can be used with
 *  MissionManager.
 */
class WaypointMission : public MissionBase
{
public:
  WaypointMission(Vehicle* vehicle = 0);
  ~WaypointMission();

  VehicleCallBackHandler wayPointEventCallback;
  VehicleCallBackHandler wayPointCallback;

  /*! @brief
   *
   *  init waypoint mission settings
   *
   *  @param Info action command from DJI_ControllerCMD.h
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void init(WayPointInitSettings* Info = 0, VehicleCallBack callback = 0,
            UserData userData = 0);
  /*! @brief
   *
   *  init waypoint mission settings
   *
   *  @param Info action command from DJI_ControllerCMD.h
   *  @param timeout timeout to wait for ACK
   */
  ACK::ErrorCode init(WayPointInitSettings* Info, int timer);
  /*! @brief
   *
   *  start the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void start(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  start the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode start(int timer);
  /*! @brief
   *
   *  stop the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void stop(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  stop the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode stop(int timer);
  /*! @brief
   *
   *  pause the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void pause(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  pause the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode pause(int timer);
  /*! @brief
   *
   *  resume the waypt mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void resume(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  resume the waypt mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode resume(int timer);
  /*! @brief
   *
   *  setting waypt init data
   *
   *  @param value user specified WayPointInitData
   */
  void setInfo(const WayPointInitSettings& value);
  /*!
   * @brief Read WayPoint mission settings from the flight controller
   *
   * @return Information about uploaded WayPoint mission, error if
   * mission settings not uploaded.
   *
   */
  ACK::WayPointInit getWaypointSettings(int timer);
  /*!
   * @brief Read WayPoint mission settings from the flight controller
   *
   * Information about uploaded WayPoint mission, error if
   * mission settings not uploaded will be handled in a user defined or
   * default callback.
   */
  void getWaypointSettings(VehicleCallBack callback, UserData userData);
  /*!
   * @brief Read WayPoint index settings from the flight controller
   *
   * @return Information about uploaded WayPoint index, error if
   * index not uploaded.
   */
  ACK::WayPointIndex getIndex(uint8_t index, int timer);
  /*!
   * @brief Read WayPoint index settings from the flight controller
   *
   * Information about uploaded WayPoint index, error if
   * index not uploaded will be handled in a user defined or
   * default callback.
   */
  void getIndex(uint8_t index, VehicleCallBack callback, UserData userData);
  /*! @brief
   *
   *  setting waypt data to the waypt container with specified idx
   *
   *  @param value user specified WayPointData
   *  @param pos the index of the waypt
   */
  void setIndex(WayPointSettings* value, size_t pos);
  /*! @brief
   *
   *  setting waypt init data
   *
   *  @param value user specified WayPointInitData
   */
  bool uploadIndexData(WayPointSettings* data, VehicleCallBack callback = 0,
                       UserData userData = 0);
  /*! @brief
   *
   *  setting waypt init data
   *
   *  @param value user specified WayPointInitData
   *  @param timer timeout to wait for ACK
   */
  ACK::WayPointIndex uploadIndexData(WayPointSettings* data, int timer);
  /*! @brief
   *
   *  getting waypt idle velocity
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void readIdleVelocity(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  getting waypt idle velocity
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode readIdleVelocity(int timeout);
  /*! @brief
   *
   *  setting waypt idle velocity
   *
   *  @param meterPreSecond specified velocity
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void updateIdleVelocity(float32_t       meterPreSecond,
                          VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  setting waypt idle velocity
   *
   *  @param meterPreSecond specified velocity
   *  @param timer timeout to wait for ACK
   */
  ACK::WayPointVelocity updateIdleVelocity(float32_t meterPreSecond,
                                           int       timeout);
  /*! @brief
   *
   *  A callback function for setting idle velocity non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void idleVelocityCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                   UserData userData);
  /*! @brief
   *
   *  A callback function for reading initialization data non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void getWaypointSettingsCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                   UserData userData);
  /*! @brief
   *
   *  A callback function for getting waypoint information for a specified index (non-blocking call)
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void getIndexCallback(Vehicle* vehicle, RecvContainer recvFrame,
				   UserData userData);
  /*! @brief
   *
   *  A callback function for uploading waypt index non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void uploadIndexDataCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                      UserData userData);
  /*! @brief
   *
   *  Set waypoint push data callback
   *
   *  @param callback callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  void setWaypointEventCallback(VehicleCallBack callback, UserData userData);
  /*! @brief
   *
   *  Set waypoint callback
   *
   *  @param callback callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  void setWaypointCallback(VehicleCallBack callback, UserData userData);

private:
  WayPointInitSettings info;
  WayPointSettings*    index;
};

} // namespace OSDK
} // namespace DJI

#endif // DJI_WAYPOINT_H
