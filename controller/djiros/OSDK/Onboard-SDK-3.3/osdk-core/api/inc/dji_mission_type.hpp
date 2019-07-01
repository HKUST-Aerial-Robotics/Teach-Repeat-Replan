/** @file dji_mission_type.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Mission related data struct for DJI OSDK library
 *
 *  @copyright 2017 DJI. All rights reserved.
 */

#ifndef ONBOARDSDK_DJI_MISSION_TYPE_H
#define ONBOARDSDK_DJI_MISSION_TYPE_H

#include "dji_type.hpp"

namespace DJI
{

namespace OSDK
{
// clang-format off
/**********Mission structs/Enums***********/

#pragma pack(1)

/**
 * @brief HotPoint Mission Initialization settings
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct HotPointSettings
{
  uint8_t   version;   /*!< Reserved, kept as 0 */
  float64_t latitude;  /*!< Latitude (radian) */
  float64_t longitude; /*!< Longitude (radian) */
  float64_t height;    /*!< Altitude (relative altitude from takeoff point */
  float64_t radius;    /*!< Radius (5m~500m) */
  float32_t yawRate;   /*!< Angle rate (0~30°/s) */
  uint8_t   clockwise; /*!< 0->fly in counter-clockwise direction, 1->clockwise direction */
  uint8_t startPoint;  /*!< Start point position <br>*/
  /*!< 0: north to the hot point <br>*/
  /*!< 1: south to the hot point <br>*/
  /*!< 2: west to the hot point <br>*/
  /*!< 3: east to the hot point <br>*/
  /*!< 4: from current position to nearest point on the hot point */
  uint8_t yawMode; /*!< Yaw mode <br>*/
  /*!< 0: point to velocity direction <br>*/
  /*!< 1: face inside <br>*/
  /*!< 2: face ouside <br>*/
  /*!< 3: controlled by RC <br>*/
  /*!< 4: same as the starting yaw<br> */
  uint8_t reserved[11]; /*!< Reserved */
} HotPointSettings;     // pack(1)

/**
 * @brief Waypoint Mission Initialization settings
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointInitSettings
{
  uint8_t   indexNumber; /*!< Total number of waypoints <br>*/
  float32_t maxVelocity; /*!< Maximum speed joystick input(2~15m) <br>*/
  float32_t idleVelocity; /*!< Cruising Speed */
  /*!< (without joystick input, no more than vel_cmd_range) */
  uint8_t finishAction; /*!< Action on finish <br>*/
  /*!< 0: no action <br>*/
  /*!< 1: return to home <br>*/
  /*!< 2: auto landing <br>*/
  /*!< 3: return to point 0 <br>*/
  /*!< 4: infinite mode， no exit <br>*/
  uint8_t executiveTimes; /*!< Function execution times <br>*/
  /*!< 1: once <br>*/
  /*!< 2: twice <br>*/
  uint8_t yawMode; /*!< Yaw mode <br>*/
  /*!< 0: auto mode(point to next waypoint) <br>*/
  /*!< 1: lock as an initial value <br>*/
  /*!< 2: controlled by RC <br>*/
  /*!< 3: use waypoint's yaw(tgt_yaw) */
  uint8_t traceMode; /*!< Trace mode <br>*/
  /*!< 0: point to point, after reaching the target waypoint hover, 
   * complete waypoints action (if any), 
   * then fly to the next waypoint <br>
   * 1: Coordinated turn mode, smooth transition between waypoints,
   * no waypoints task <br>
   */
  uint8_t RCLostAction; /*!< Action on rc lost <br>*/
  /*!< 0: exit waypoint and failsafe <br>*/
  /*!< 1: continue the waypoint <br>*/
  uint8_t gimbalPitch; /*!< Gimbal pitch mode <br>*/
  /*!< 0: free mode, no control on gimbal <br>*/
  /*!< 1: auto mode, Smooth transition between waypoints <br>*/
  float64_t latitude;     /*!< Focus latitude (radian) */
  float64_t longitude;    /*!< Focus longitude (radian) */
  float32_t altitude;     /*!< Focus altitude (relative takeoff point height) */
  uint8_t   reserved[16]; /*!< Reserved, must be set to 0 */

} WayPointInitSettings; // pack(1)

/**
 * @brief Waypoint settings for individual waypoints being added to the mission
 * @details This is one of the few structs in the OSDK codebase that
 * is used in both a sending and a receiving API.
 */
typedef struct WayPointSettings
{
  uint8_t   index;     /*!< Index to be uploaded */
  float64_t latitude;  /*!< Latitude (radian) */
  float64_t longitude; /*!< Longitude (radian) */
  float32_t altitude;  /*!< Altitude (relative altitude from takeoff point) */
  float32_t damping; /*!< Bend length (effective coordinated turn mode only) */
  int16_t   yaw;     /*!< Yaw (degree) */
  int16_t   gimbalPitch; /*!< Gimbal pitch */
  uint8_t   turnMode;    /*!< Turn mode <br> */
  /*!< 0: clockwise <br>*/
  /*!< 1: counter-clockwise <br>*/
  uint8_t reserved[8]; /*!< Reserved */
  uint8_t hasAction;   /*!< Action flag <br>*/
  /*!< 0: no action <br>*/
  /*!< 1: has action <br>*/
  uint16_t actionTimeLimit;      /*!< Action time limit */
  uint8_t  actionNumber : 4;     /*!< Total number of actions */
  uint8_t  actionRepeat : 4;     /*!< Total running times */
  uint8_t  commandList[16];      /*!< Command list */
  uint16_t commandParameter[16]; /*!< Command parameters */
} WayPointSettings;              // pack(1)

/**
 * @brief WayPoint Push Data Incident Type enumerator
 */
//! @note can be separated by the first bytes of data
typedef enum WayPointIncidentType {
  NAVI_UPLOAD_FINISH,
  NAVI_MISSION_FINISH,
  NAVI_MISSION_WP_REACH_POINT
} WayPointIncidentType;

/**
 * @brief Waypoint Mission Finish Event Push Data
 */
typedef struct WayPointFinishData
{
  uint8_t  incident_type; /*! see WayPointIncidentType */
  uint8_t  repeat;
  uint16_t reserved_1;
  uint16_t reserved_2;
} WayPointFinishData; // pack(1)

#pragma pack()
// clang-format on
} // OSDK

} // DJI

#endif // ONBOARDSDK_DJI_MISSION_TYPE_H
