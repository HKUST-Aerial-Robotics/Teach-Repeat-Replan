/** @file dji_hotpoint.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of HotPoint (Point of Interest) Missions for DJI OSDK
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#ifndef DJI_HOTPOINT_H
#define DJI_HOTPOINT_H

#include "dji_mission_base.hpp"
#include "dji_telemetry.hpp"

namespace DJI
{
namespace OSDK
{

/*! @brief APIs for Hotpoint (Point of Interest) Missions
 *
 * @details This class inherits from MissionBase and can be used with
 * MissionManager.
 */
class HotpointMission : public MissionBase
{
public:
#pragma pack(1)
  typedef struct YawRate
  {
    uint8_t   clockwise;
    float32_t yawRate;
  } YawRate;
#pragma pack()

  enum View
  {
    VIEW_NORTH  = 0,
    VIEW_SOUTH  = 1,
    VIEW_WEST   = 2,
    VIEW_EAST   = 3,
    VIEW_NEARBY = 4
  };

  enum YawMode
  {
    YAW_AUTO    = 0,
    YAW_INSIDE  = 1,
    YAW_OUTSIDE = 2,
    YAW_CUSTOM  = 3,
    YAW_STATIC  = 4,
  };

  /*! @note API functions
   *  @attention difference between set and update <br>
   *  Set functions only change the HotPoint data in this class,
   *  Update functions will change the Mission status.
   *  In other words: drone will response update functions immediately.
   */
public:
  HotpointMission(Vehicle* vehicle = 0);
  ~HotpointMission();

  VehicleCallBackHandler hotPointCallback;

  /*! @brief
   *
   *  init hotpoint default data
   *
   */
  void initData();
  /*! @brief
   *
   *  getting hotpoint data
   *
   */
  HotPointSettings getData() const;
  /*! @brief
   *
   *  start the hotpoint mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void start(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  start the hotpoint mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode start(int timer);
  /*! @brief
   *
   *  stop the hotpoint mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void stop(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  stop the hotpoint mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode stop(int timer);
  /*! @brief
   *
   *  pause the hotpoint mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void pause(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  pause the hotpoint mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode pause(int timer);
  /*! @brief
   *
   *  resume the hotpoint mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void resume(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  resume the hotpoint mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode resume(int timer);
  /*! @brief
   *
   *  update yaw rate and orientation of hotpoint mission
   *
   *  @param Data specified yaw rate and orientation
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void updateYawRate(YawRate& Data, VehicleCallBack callback = 0,
                     UserData userData = 0);
  /*! @brief
   *
   *  update yaw rate and orientation of hotpoint mission
   *
   *  @param Data specified yaw rate and orientation
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode updateYawRate(YawRate& Data, int timer);
  /*! @brief
   *
   *  update yaw rate and orientation of hotpoint mission
   *
   *  @param yawRate specified yaw rate
   *  @param isClockwise specified orientation
   *  @param timer timeout to wait for ACK
   */
  void updateYawRate(float32_t yawRate, bool isClockwise,
                     VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  update radius of hotpoint mission
   *
   *  @param meter radius
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void updateRadius(float32_t meter, VehicleCallBack callback = 0,
                    UserData userData = 0);
  /*! @brief
   *
   *  update radius of hotpoint mission
   *
   *  @param meter radius
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode updateRadius(float32_t meter, int timer);
  /*! @brief
   *
   *  reset yaw of hotpoint mission
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void resetYaw(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  reset yaw of hotpoint mission
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::ErrorCode resetYaw(int timer);
  /*! @brief
   *
   *  Read hotpoint mission information from flight controller
   *
   *  @param callback callback function
   *  @param userData user data (void ptr)
   */
  void getHotpointSettings(VehicleCallBack callback = 0, UserData userData = 0);
  /*! @brief
   *
   *  Read hotpoint mission information from flight controller
   *
   *  @param timer timeout to wait for ACK
   */
  ACK::HotPointRead getHotpointSettings(int timer);
  /*! @brief
   *
   *  A callback function for start non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void startCallback(RecvContainer recvFrame, UserData userData);
  /*! @brief
   *
   *  A callback function for read non-blocking calls
   *
   *  @param recvFrame the data comes with the callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  static void getHotpointSettingsCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
		                          UserData userData);
  /*! @brief
   *
   *  Set hotpoint callback
   *
   *  @param callback callback function
   *  @param userData a void ptr that user can manipulate inside the callback
   */
  void setHotpointCallback(VehicleCallBack callback, UserData userData);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param data HotPointSettings
   */
  void setData(HotPointSettings* data);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param longitude longitude
   *  @param latitude latitude
   *  @param altitude altitude
   */
  void setHotPoint(float64_t longitude, float64_t latitude, float64_t altitude);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param gps gps
   */
  void setHotPoint(Telemetry::GlobalPosition gps);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param meter radius in meter
   */
  void setRadius(float64_t meter);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param degree yawrate in degree/sec
   */
  void setYawRate(float32_t degree);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param isClockwise isClockwise
   */
  void setClockwise(bool isClockwise);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param view check View struct
   */
  void setCameraView(View view);
  /*! @brief
   *
   *  Set hotpoint data for initialization purpose
   *
   *  @param mode check YawMode struct
   */
  void setYawMode(YawMode mode);

private:
  HotPointSettings hotPointData;
};

} // namespace OSDK
} // namespace DJI

#endif // DJI_HOTPOINT_H
