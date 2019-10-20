/** @file dji_status.hpp
 *  @version 3.3
 *  @date June 2017
 *
 *  @brief
 *
 *  Status information for DJI Vehicle
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJI_STATUS_H
#define DJI_STATUS_H

// clang-format off
namespace DJI
{
namespace OSDK
{
/*!
 * @brief info about vehicle
 */
namespace VehicleStatus
{
/*!
 * @brief "Mode" of the vehicle's state machine, as displayed on DJI Go
 * @details Available through Telemetry::TOPIC_STATUS_DISPLAYMODE
 */
namespace DisplayMode
{
enum
{
  /*! This mode requires the user to manually
   * control the aircraft to remain stable in air. */
  MODE_MANUAL_CTRL = 0,
  /*! In this mode, the aircraft can keep
   * attitude stabilization and only use the
   * barometer for positioning to control the altitude. <br>
   * The aircraft can not autonomously locate and hover stably.*/
  MODE_ATTITUDE   = 1,
  MODE_RESERVED_2 = 2,
  MODE_RESERVED_3 = 3,
  MODE_RESERVED_4 = 4,
  MODE_RESERVED_5 = 5,
  /*! The aircraft is in normal GPS mode. <br>
   * In normal GPS mode, the aircraft can
   * autonomously locate and hover stably. <br>
   *  The sensitivity of the aircraft to the
   *  command response is moderate.
   */
  MODE_P_GPS      = 6,
  MODE_RESERVED_7 = 7,
  MODE_RESERVED_8 = 8,
  /*! In hotpoint mode */
  MODE_HOTPOINT_MODE = 9,
  /*! In this mode, user can push the throttle
   * stick to complete stable take-off. */
  MODE_ASSISTED_TAKEOFF = 10,
  /*! In this mode, the aircraft will autonomously
   * start motor, ascend and finally hover. */
  MODE_AUTO_TAKEOFF = 11,
  /*! In this mode, the aircraft can land autonomously. */
  MODE_AUTO_LANDING = 12,
  MODE_RESERVED_13  = 13,
  MODE_RESERVED_14  = 14,
  /*! In this mode, the aircraft can antonomously return the
   * last recorded Home Point. <br>
   * There are three types of this mode: Smart RTH(Return-to-Home),
   * Low Batterry RTH, and Failsafe RTTH.  */
  MODE_NAVI_GO_HOME = 15,
  MODE_RESERVED_16  = 16,
  /*! In this mode, the aircraft is controled by SDK API. <br>
   * User can directly define the control mode of horizon
   * and vertical directions and send control datas to aircraft. */
  MODE_NAVI_SDK_CTRL  = 17,
  MODE_RESERVED_18    = 18,
  MODE_RESERVED_19    = 19,
  MODE_RESERVED_20    = 20,
  MODE_RESERVED_21    = 21,
  MODE_RESERVED_22    = 22,
  MODE_RESERVED_23    = 23,
  MODE_RESERVED_24    = 24,
  MODE_RESERVED_25    = 25,
  MODE_RESERVED_26    = 26,
  MODE_RESERVED_27    = 27,
  MODE_RESERVED_28    = 28,
  MODE_RESERVED_29    = 29,
  MODE_RESERVED_30    = 30,
  MODE_RESERVED_31    = 31,
  MODE_RESERVED_32    = 32,
  /*! drone is forced to land, might due to low battery */
  MODE_FORCE_AUTO_LANDING = 33,
  MODE_RESERVED_34        = 34,
  MODE_RESERVED_35        = 35,
  MODE_RESERVED_36        = 36,
  MODE_RESERVED_37        = 37,
  MODE_RESERVED_38        = 38,
  MODE_RESERVED_39        = 39,
  /*! drone will search for the last position where the rc is not lost */
  MODE_SEARCH_MODE = 40,
  /*! Mode for motor starting. <br>
   * Every time user unlock the motor, this will be the first mode. */
  MODE_ENGINE_START = 41,
  MODE_RESERVED_42  = 42,
  MODE_RESERVED_43  = 42,
};
} // namespace DisplayMode

/*!
 * @brief Enums for various landing gear states.
 * @details Available through broadcast (SDKInfo)/subscribe (Telemetry::TOPIC_STATUS_LANDINGGEAR)
 */
namespace LandingGearMode
{
enum
{
  LANDING_GEAR_UNDEFINED             = 0,
  LANDING_GEAR_DOWN                  = 1,
  LANDING_GEAR_UP_TO_DOWN            = 2,
  LANDING_GEAR_UP                    = 3,
  LANDING_GEAR_DOWN_TO_UP            = 4,
  LANDING_GEAR_HOLD                  = 5,
  LANDING_GEAR_PACKED                = 6,
  LANDING_GEAR_PACKING_IN_PROGRESS   = 7,
  LANDING_GEAR_UNPACKING_IN_PROGRESS = 8,
};
} // namespace LandingGearMode

/*!
 * @brief Enums for describing the current control mode the aircraft is in.
 * @details Available through broadcast (SDKInfo)/subscribe (Telemetry::TOPIC_CONTROL_DEVICE)
 */
namespace CtlrMode
{
enum
{
  CTRL_MODE_ATTI_STOP                   = 0,
  CTRL_MODE_HORIZ_ANG_VERT_VEL_YAW_ANG  = 1,
  CTRL_MODE_HORIZ_ANG_VERT_VEL_YAW_RATE = 2,
  CTRL_MODE_HORIZ_VEL_VERT_VEL_YAW_ANG  = 3,
  CTRL_MODE_HORIZ_VEL_VERT_VEL_YAW_RATE = 4,
  CTRL_MODE_HORIZ_POS_VERT_VEL_YAW_ANG  = 5,
  CTRL_MODE_HORIZ_POS_VERT_VEL_YAW_RATE = 6,
  CTRL_MODE_HORIZ_ANG_VERT_POS_YAW_ANG  = 7,
  CTRL_MODE_HORIZ_ANG_VERT_POS_YAW_RATE = 8,
  CTRL_MODE_HORIZ_VEL_VERT_POS_YAW_ANG  = 9,
  CTRL_MODE_HORIZ_VEL_VERT_POS_YAW_RATE = 10,
  CTRL_MODE_HORIZ_POS_VERT_POS_YAW_ANG  = 11,
  CTRL_MODE_HORIZ_POS_VERT_POS_YAW_RATE = 12,
  CTRL_MODE_HORIZ_ANG_VERT_THR_YAW_ANG  = 13,
  CTRL_MODE_HORIZ_ANG_VERT_THR_YAW_RATE = 14,
  CTRL_MODE_HORIZ_VEL_VERT_THR_YAW_ANG  = 15,
  CTRL_MODE_HORIZ_VEL_VERT_THR_YAW_RATE = 16,
  CTRL_MODE_HORIZ_POS_VERT_THR_YAW_ANG  = 17,
  CTRL_MODE_HORIZ_POS_VERT_THR_YAW_RATE = 18,
  CTRL_MODE_HORIZ_PAL_VERT_VEL_YAW_RATE = 19,
  CTRL_MODE_HORIZ_PAL_VERT_POS_YAW_RAT  = 20,
  CTRL_MODE_HORIZ_PAL_VERT_THR_YAW_RATE = 21,
  CTRL_MODE_GPS_ATII_CTRL_CL_YAW_RATE   = 97, //!< @note unused
  CTRL_MODE_GPS_ATTI_CTRL_YAW_RATE      = 98, //!< @note unused
  CTRL_MODE_ATTI_CTRL_YAW_RATE          = 99, //!< @note unused
  CTRL_MODE_ATTI_CTRL_STOP              = 100,
  CTRL_MODE_MODE_NOT_SUPPORTED          = 0xFF //!< @note unused
};
} // namespace CtrlMode

/*! @brief Flight Status enum
 * @note this is a DataBroadcast/DataSubscription telemetry message.
 *
 */
namespace FlightStatus
{
enum
{
  STOPED      = 0,
  ON_GROUND   = 1,
  IN_AIR      = 2
};
} // namespace FlightStatus

/*
 * Flight status supported by M100_31
 */
namespace M100FlightStatus
{
enum
{
  ON_GROUND_STANDBY  = 1,
  TAKEOFF            = 2,
  IN_AIR_STANDBY     = 3,
  LANDING            = 4, 
  FINISHING_LANDING  = 5
};
} // namespace M100FlightStatus

} // namespace VehicleStatus
} // namespace OSDK
} // namespace DJI

// clang-format on
#endif /* DJI_STATUS_H */
