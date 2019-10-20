/** @file dji_ack.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK ACK parsing
 *  @brief ACK error API getError and getErrorCodeMessage
 *  to decode received ACK(s).
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_ack.hpp"
#include "dji_log.hpp"
#include <string.h>

const bool DJI::OSDK::ACK::SUCCESS = 0;
const bool DJI::OSDK::ACK::FAIL    = 1;

namespace DJI
{
namespace OSDK
{

const std::pair<const uint32_t, const char*> commonData[] = {
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR,
                 (const char*)"ACK_NO_RESPONSE_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::KEY_ERROR,
                 (const char*)"ACK_KEY_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::NO_AUTHORIZATION_ERROR,
                 (const char*)"ACK_NO_AUTHORIZATION_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::NO_RIGHTS_ERROR,
                 (const char*)"ACK_NO_RIGHTS_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::SUCCESS,
                 (const char*)"ACK_SUCCESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::CommonACK::START_MOTOR_FAIL_MOTOR_STARTED,
    (const char*)"START_MOTOR_FAIL_MOTOR_STARTED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::CommonACK::MOTOR_FAIL_COMPASS_ABNORMAL,
    (const char*)"MOTOR_FAIL_COMPASS_ABNORMAL\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::CommonACK::MOTOR_FAIL_ASSISTANT_PROTECTED,
    (const char*)"USB_CABLE_CONNECTED_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::MOTOR_FAIL_DEVICE_LOCKED,
                 (const char*)"MOTOR_FAIL_DEVICE_LOCKED\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::MOTOR_FAIL_IMU_CALIBRATING,
                 (const char*)"MOTOR_FAIL_IMU_CALIBRATING\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::CommonACK::MOTOR_FAIL_M600_BAT_TOO_FEW,
    (const char*)"MISSING_BATTERIES\n"),
  std::make_pair(OpenProtocol::ErrorCode::CommonACK::MOTOR_FAIL_NOT_ACTIVATED,
                 (const char*)"NOT_ACTIVATED_ERROR\n")
};

const std::map<const uint32_t, const char*>
ACK::createCommonErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    commonData, commonData + sizeof commonData / sizeof commonData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> activateData[] = {
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::SUCCESS,
                 (const char*)"ACTIVATE_SUCCESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::ACCESS_LEVEL_ERROR,
                 (const char*)"ACCESS_LEVEL_ERROR\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ActivationACK::DJIGO_APP_NOT_CONNECTED,
    (const char*)"DJIGO_APP_NOT_CONNECTED_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::ENCODE_ERROR,
                 (const char*)"ENCODE_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::NETWORK_ERROR,
                 (const char*)"NETWORK_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::NEW_DEVICE_ERROR,
                 (const char*)"NEW_DEVICE_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::OSDK_VERSION_ERROR,
                 (const char*)"OSDK_VERSION_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::PARAMETER_ERROR,
                 (const char*)"PARAMETER_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ActivationACK::SERVER_ACCESS_REFUSED,
                 (const char*)"SERVER_ACCESS_REFUSED\n")
};

const std::map<const uint32_t, const char*>
ACK::createActivateErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    activateData, activateData + sizeof activateData / sizeof activateData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> subscribeData[] = {
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::SUCCESS,
                 (const char*)"SUBSCRIBER_SUCCESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::ILLEGAL_FREQUENCY,
                 (const char*)"SUBSCRIBER_ILLEGAL_FREQUENCY\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::ILLEGAL_INPUT,
                 (const char*)"SUBSCRIBER_ILLEGAL_INPUT\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::ILLEGAL_UID,
                 (const char*)"SUBSCRIBER_ILLEGAL_UID\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::SubscribeACK::INPUT_SEGMENTATION_FAULT,
    (const char*)"SUBSCRIBER_INPUT_SEGMENTATION_FAULT\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::INTERNAL_ERROR_0X09,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0X09\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::INTERNAL_ERROR_0X4A,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0X4A\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::INTERNAL_ERROR_0X50,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0X50\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::INTERNAL_ERROR_0XFF,
                 (const char*)"SUBSCRIBER_INTERNAL_ERROR_0XFF\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::MULTIPLE_SUBSCRIBE,
                 (const char*)"SUBSCRIBER_MULTIPLE_SUBSCRIBE\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PACKAGE_ALREADY_EXISTS,
                 (const char*)"SUBSCRIBER_PACKAGE_ALREADY_EXISTS\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PACKAGE_DOES_NOT_EXIST,
                 (const char*)"SUBSCRIBER_PACKAGE_DOES_NOT_EXIST\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PACKAGE_EMPTY,
                 (const char*)"SUBSCRIBER_PACKAGE_EMPTY\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PACKAGE_OUT_OF_RANGE,
                 (const char*)"SUBSCRIBER_PACKAGE_OUT_OF_RANGE\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PACKAGE_TOO_LARGE,
                 (const char*)"SUBSCRIBER_PACKAGE_TOO_LARGE\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PAUSED,
                 (const char*)"SUBSCRIBER_PAUSED\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PERMISSION_DENY,
                 (const char*)"SUBSCRIBER_PERMISSION_DENY\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::PIPELINE_OVERFLOW,
                 (const char*)"SUBSCRIBER_PIPELINE_OVERFLOW\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::RESUMED,
                 (const char*)"SUBSCRIBER_RESUMED\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::SOUCE_DEVICE_OFFLINE,
                 (const char*)"SUBSCRIBER_SOUCE_DEVICE_OFFLINE\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::VERSION_DOES_NOT_MATCH,
                 (const char*)"SUBSCRIBER_VERSION_DOES_NOT_MATCH\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::VERSION_UNKNOWN_ERROR,
                 (const char*)"SUBSCRIBER_VERSION_UNKNOWN_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::SubscribeACK::VERSION_VERSION_TOO_FAR,
                 (const char*)"SUBSCRIBER_VERSION_VERSION_TOO_FAR\n")
};

const std::map<const uint32_t, const char*>
ACK::createSubscribeErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    subscribeData,
    subscribeData + sizeof subscribeData / sizeof subscribeData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> setControlData[] = {
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetControl::IOC_OBTAIN_CONTROL_ERROR,
    (const char*)"IOC_OBTAIN_CONTROL_ERROR\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_IN_PROGRESS,
    (const char*)"OBTAIN_CONTROL_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_SUCCESS,
    (const char*)"OBTAIN_CONTROL_SUCCESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::SetControl::RC_MODE_ERROR,
                 (const char*)"RC_MODE_ERROR\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetControl::RC_NEED_MODE_F,
    (const char*)"RC_NEED_MODE_F\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetControl::RC_NEED_MODE_P,
    (const char*)"RC_NEED_MODE_P\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::SetControl::
                   RELEASE_CONTROL_IN_PROGRESS,
                 (const char*)"RELEASE_CONTROL_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetControl::RELEASE_CONTROL_SUCCESS,
    (const char*)"RELEASE_CONTROL_SUCCESS\n")
};

const std::map<const uint32_t, const char*>
ACK::createSetControlErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    setControlData,
    setControlData + sizeof setControlData / sizeof setControlData[0]);
  return errorCodeMap;
}

/*
 * SetArm supported only in Matrice 100
 */
const std::pair<const uint32_t, const char*> setArmData[] = {
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::SetArm::SUCCESS,
                 (const char*)"SET_ARM_SUCCESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetArm::AIRCRAFT_IN_AIR_ERROR,
    (const char*)"SET_ARM_AIRCRAFT_IN_AIR_ERROR\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetArm::ALREADY_ARMED_ERROR,
    (const char*)"SET_ARM_ALREADY_ARMED_ERROR\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::SetArm::OBTAIN_CONTROL_NEEDED_ERROR,
    (const char*)"SET_ARM_OBTAIN_CONTROL_NEEDED_ERROR\n")
};

const std::map<const uint32_t, const char*>
ACK::createSetArmErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    setArmData, setArmData + sizeof setArmData / sizeof setArmData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> taskData[] = {
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::SUCCESS,
                 (const char*)"CONTROLLER_TASK_SUCCESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::ALREADY_PACKED,
                 (const char*)"CONTROLLER_ALREADY_PACKED\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::ALREADY_RUNNING,
                 (const char*)"CONTROLLER_ALREADY_RUNNING\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::BAD_GPS,
                 (const char*)"CONTROLLER_BAD_GPS\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::BAD_SENSOR,
                 (const char*)"CONTROLLER_BAD_SENSOR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::GIMBAL_MOUNTED,
                 (const char*)"CONTROLLER_GIMBAL_MOUNTED\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::INVAILD_COMMAND,
                 (const char*)"CONTROLLER_INVAILD_COMMAND\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::IN_AIR,
                 (const char*)"CONTROLLER_IN_AIR"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::IN_SIMULATOR_MODE,
                 (const char*)"CONTROLLER_IN_SIMULATOR_MODE\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::MOTOR_OFF,
                 (const char*)"CONTROLLER_MOTOR_OFF\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::MOTOR_ON,
                 (const char*)"CONTROLLER_MOTOR_ON\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::NOT_IN_AIR,
                 (const char*)"CONTROLLER_NOT_IN_AIR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::NOT_RUNNING,
                 (const char*)"CONTROLLER_NOT_RUNNING\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::NO_HOMEPOINT,
                 (const char*)"CONTROLLER_NO_HOMEPOINT\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::NO_LANDING_GEAR,
                 (const char*)"CONTROLLER_NO_LANDING_GEAR\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::Task::NO_PACKED,
                 (const char*)"CONTROLLER_NO_PACKED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::ControlACK::Task::PACKED_MODE_NOT_SUPPORTED,
    (const char*)"CONTROLLER_PACKED_MODE_NOT_SUPPORTED\n")
};

const std::map<const uint32_t, const char*>
ACK::createTaskErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    taskData, taskData + sizeof taskData / sizeof taskData[0]);
  return errorCodeMap;
}

/*
 * Supported in Matrice 100
 */
const std::pair<const uint32_t, const char*> M100TaskData[] = {
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::M100Task::SUCCESS,
                 (const char*)"CONTROLLER_SUCCESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::ControlACK::M100Task::FAIL,
                 (const char*)"CONTROLLER_FAIL\n")
};

const std::map<const uint32_t, const char*>
ACK::createM100TaskErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    M100TaskData, M100TaskData + sizeof M100TaskData / sizeof M100TaskData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> missionData[] = {
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::AT_NO_FLY_ZONE,
                 (const char*)"MISSION_AT_NO_FLY_ZONE\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::BAD_GPS,
                 (const char*)"MISSION_BAD_GPS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::BEGGINER_MODE_NOT_SUPPORTED,
    (const char*)"MISSION_BEGGINER_MODE_NOT_SUPPORTED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::CLOSE_IOC_REQUIRED,
    (const char*)"MISSION_CLOSE_IOC_REQUIRED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::CONDITIONS_NOT_SATISFIED,
    (const char*)"MISSION_CONDITIONS_NOT_SATISFIED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::CROSSING_NO_FLY_ZONE,
    (const char*)"MISSION_CROSSING_NO_FLY_ZONE\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::INVALID_COMMAND,
                 (const char*)"MISSION_INVALID_COMMAND\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::INVALID_PARAMETER,
                 (const char*)"MISSION_INVALID_PARAMETER"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::IN_PROGRESS,
                 (const char*)"MISSION_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::LANDING_IN_PROGRESS,
    (const char*)"MISSION_LANDING_IN_PROGRESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::LOW_BATTERY,
                 (const char*)"MISSION_LOW_BATTERY\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::NOT_INITIALIZED,
                 (const char*)"MISSION_NOT_INITIALIZED\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::NOT_RUNNING,
                 (const char*)"MISSION_NOT_RUNNING\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::NOT_SUPPORTED,
                 (const char*)"MISSION_NOT_SUPPORTED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::OBTAIN_CONTROL_REQUIRED,
    (const char*)"MISSION_OBTAIN_CONTROL_REQUIRED\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::OTHER_MISSION_RUNNING,
    (const char*)"OTHER_MISSION_RUNNING\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::RC_NOT_IN_MODE_F,
                 (const char*)"MISSION_RC_NOT_IN_MODE_F\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::RRETURN_HOME_IN_PROGRESS,
    (const char*)"MISSION_RRETURN_HOME_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::START_MOTORS_IN_PROGRESS,
    (const char*)"MISSION_START_MOTORS_IN_PROGRESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::SUCCESS,
                 (const char*)"MISSION_SUCCESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::TAKE_OFF_IN_PROGRESS,
    (const char*)"MISSION_TAKE_OFF_IN_PROGRESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::TASK_TIMEOUT,
                 (const char*)"MISSION_TASK_TIMEOUT\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::TOO_FAR_FROM_CURRENT_POSITION,
    (const char*)"MISSION_TOO_FAR_FROM_CURRENT_POSITION\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::TOO_FAR_FROM_HOME,
                 (const char*)"MISSION_TOO_FAR_FROM_HOME\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::TOO_HIGH,
                 (const char*)"MISSION_TOO_HIGH\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::TOO_LOW,
                 (const char*)"MISSION_TOO_LOW\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::UNKNOWN_ERROR,
                 (const char*)"MISSION_UNKNOWN_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Common::UNRECORDED_HOME,
                 (const char*)"MISSION_UNRECORDED_HOME\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::VEHICLE_DID_NOT_TAKE_OFF,
    (const char*)"MISSION_VEHICLE_DID_NOT_TAKE_OFF\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Common::WRONG_WAYPOINT_INDEX,
    (const char*)"MISSION_WRONG_WAYPOINT_INDEX\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Follow::CUTOFF_TIME_OVERFLOW,
    (const char*)"FOLLOW_MISSION_CUTOFF_TIME_OVERFLOW\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::Follow::GIMBAL_PITCH_ANGLE_OVERFLOW,
    (const char*)"FOLLOW_MISSION_GIMBAL_PITCH_ANGLE_OVERFLOW\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::Follow::
                   TOO_FAR_FROM_YOUR_POSITION_LACK_OF_RADIO_CONNECTION,
                 (const char*)"FOLLOW_MISSION_TOO_FAR_FROM_YOUR_POSITION_LACK_"
                              "OF_RADIO_CONNECTION\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::HotPoint::FAILED_TO_PAUSE,
                 (const char*)"HOTPOINT_MISSION_FAILED_TO_PAUSE\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::INVALID_DIRECTION,
    (const char*)"HOTPOINT_MISSION_INVALID_DIRECTION\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::
      INVALID_LATITUDE_OR_LONGITUTE,
    (const char*)"HOTPOINT_MISSION_INVALID_LATITUDE_OR_LONGITUTE\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::INVALID_PAREMETER,
    (const char*)"HOTPOINT_MISSION_INVALID_PAREMETER\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::HotPoint::INVALID_RADIUS,
                 (const char*)"HOTPOINT_MISSION_INVALID_RADIUS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::INVALID_START_POINT,
    (const char*)"HOTPOINT_MISSION_INVALID_VISION\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::INVALID_YAW_MODE,
    (const char*)"HOTPOINT_MISSION_INVALID_YAW_MODE\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::HotPoint::IN_PAUSED_MODE,
                 (const char*)"HOTPOINT_MISSION_IN_PAUSED_MODE\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::TOO_FAR_FROM_HOTPOINT,
    (const char*)"HOTPOINT_MISSION_TOO_FAR_FROM_HOTPOINT\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::HotPoint::YAW_RATE_OVERFLOW,
    (const char*)"HOTPOINT_MISSION_YAW_RATE_OVERFLOW\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::CHECK_FAILED,
                 (const char*)"WAYPOINT_MISSION_CHECK_FAILED\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::DATA_NOT_ENOUGH,
                 (const char*)"WAYPOINT_MISSION_DATA_NOT_ENOUGH\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::WayPoint::DISTANCE_OVERFLOW,
    (const char*)"WAYPOINT_MISSION_DISTANCE_OVERFLOW\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::INVALID_ACTION,
                 (const char*)"WAYPOINT_MISSION_INVALID_ACTION\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::INVALID_DATA,
                 (const char*)"WAYPOINT_MISSION_INVALID_DATA\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::WayPoint::INVALID_POINT_DATA,
    (const char*)"WAYPOINT_MISSION_INVALID_POINT_DATA\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::WayPoint::INVALID_VELOCITY,
    (const char*)"WAYPOINT_MISSION_INVALID_VELOCITY\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::IN_PROGRESS,
                 (const char*)"MISSION_IN_PROGRESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::NOT_IN_PROGRESS,
                 (const char*)"WAYPOINT_MISSION_NOT_IN_PROGRESS\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::WayPoint::POINTS_NOT_ENOUGH,
    (const char*)"WAYPOINT_MISSION_POINTS_NOT_ENOUGH\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::WayPoint::POINTS_TOO_CLOSE,
    (const char*)"WAYPOINT_MISSION_POINTS_TOO_CLOSE\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::POINTS_TOO_FAR,
                 (const char*)"WAYPOINT_MISSION_POINTS_TOO_FAR\n"),
  std::make_pair(
    OpenProtocol::ErrorCode::MissionACK::WayPoint::POINT_DATA_NOT_ENOUGH,
    (const char*)"WAYPOINT_MISSION_POINT_DATA_NOT_ENOUGH\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::POINT_OVERFLOW,
                 (const char*)"WAYPOINT_MISSION_POINT_OVERFLOW\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::WayPoint::TIMEOUT,
                 (const char*)"WAYPOINT_MISSION_TIMEOUT\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::IOC::TOO_CLOSE_TO_HOME,
                 (const char*)"IOC_MISSION_TOO_CLOSE_TO_HOME\n"),
  std::make_pair(OpenProtocol::ErrorCode::MissionACK::IOC::UNKNOWN_TYPE,
                 (const char*)"IOC_MISSION_UNKNOWN_TYPE\n")
};

const std::map<const uint32_t, const char*>
ACK::createMissionErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    missionData, missionData + sizeof missionData / sizeof missionData[0]);
  return errorCodeMap;
}

const std::pair<const uint32_t, const char*> mfioData[] = {
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::init::PORT_DATA_ERROR,
                 (const char*)"MFIO_INIT_PORT_DATA_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::init::PORT_MODE_ERROR,
                 (const char*)"MFIO_INIT_PORT_MODE_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::init::PORT_NUMBER_ERROR,
                 (const char*)"MFIO_INIT_PORT_NUMBER_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::init::SUCCESS,
                 (const char*)"MFIO_INIT_SUCCESS\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::init::UNKNOWN_ERROR,
                 (const char*)"RC_NEED_MODE_F\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::set::CHANNEL_ERROR,
                 (const char*)"MFIO_SET_CHANNEL_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::set::PORT_NOT_MAPPED_ERROR,
                 (const char*)"MFIO_SET_PORT_NOT_MAPPED_ERROR\n"),
  std::make_pair(OpenProtocol::ErrorCode::MFIOACK::set::SUCCESS,
                 (const char*)"MFIO_SET_SUCCESS\n")
};

const std::map<const uint32_t, const char*>
ACK::createMFIOErrorCodeMap()
{
  const std::map<const uint32_t, const char*> errorCodeMap(
    mfioData, mfioData + sizeof mfioData / sizeof mfioData[0]);
  return errorCodeMap;
}

bool
ACK::getError(ACK::ErrorCode ack)
{
  const uint8_t cmd[] = { ack.info.cmd_set, ack.info.cmd_id };

  if (ack.info.cmd_set == OpenProtocol::CMDSet::activation)
  {
    return (ack.data == OpenProtocol::ErrorCode::ActivationACK::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
  }
  else if (ack.info.cmd_set == OpenProtocol::CMDSet::broadcast)
  {
    // Push Data, no ACK
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::setControl,
                  sizeof(cmd)) == 0)
  {
    if (ack.info.buf[2] == ACK::RELEASE_CONTROL)
    { //! Data is set at buf + SET_CMD_SIZE which is buf + 2;
      // Release control was called
      return (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                            RELEASE_CONTROL_SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else if (ack.info.buf[2] == ACK::OBTAIN_CONTROL)
    {
      // Obtain control was called
      return (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                            OBTAIN_CONTROL_SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::setArm, sizeof(cmd)) == 0)
  {
    /*
     * SetArm command supported in Matrice 100
     */
    return (ack.data == OpenProtocol::ErrorCode::ControlACK::SetArm::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::control, sizeof(cmd)) ==
           0)
  {
    // Does not return an ACK
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::task, sizeof(cmd)) == 0)
  {
    if (ack.info.version != Version::M100_31)
    {
      return (ack.data == OpenProtocol::ErrorCode::ControlACK::Task::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else
    {
      //! ACKs supported in Matrice 100
      return (ack.data ==
              OpenProtocol::ErrorCode::ControlACK::M100Task::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }
  else if (ack.info.cmd_set == OpenProtocol::CMDSet::mission)
  {
    return (ack.data == OpenProtocol::ErrorCode::MissionACK::Common::SUCCESS)
             ? ACK::SUCCESS
             : ACK::FAIL;
  }
  else if (ack.info.cmd_set == OpenProtocol::CMDSet::hardwareSync)
  {
    // Verify ACK
  }
  else if (ack.info.cmd_set == OpenProtocol::CMDSet::virtualRC)
  {
    // Deprecated in 3.2.20

    // TODO implement for backward compatibility
  }
  else if (ack.info.cmd_set == OpenProtocol::CMDSet::subscribe)
  {
    if (memcmp(cmd, OpenProtocol::CMDSet::Subscribe::pauseResume,
               sizeof(cmd)) == 0)
    {
      return (ack.data == OpenProtocol::ErrorCode::SubscribeACK::PAUSED ||
              ack.data == OpenProtocol::ErrorCode::SubscribeACK::RESUMED)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else
    {
      return (ack.data == OpenProtocol::ErrorCode::SubscribeACK::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }
  else if (ack.info.cmd_set == OpenProtocol::CMDSet::mfio)
  {
    if (memcmp(cmd, OpenProtocol::CMDSet::MFIO::get, sizeof(cmd)) == 0)
    {
      return (ack.data == OpenProtocol::ErrorCode::MFIOACK::get::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
    else
    {
      return (ack.data == OpenProtocol::ErrorCode::MFIOACK::init::SUCCESS)
               ? ACK::SUCCESS
               : ACK::FAIL;
    }
  }

  return ACK::FAIL;
}

/*
 * @note Log Error Message
 */
void
ACK::getErrorCodeMessage(ACK::ErrorCode ack, const char* func)
{
  DSTATUS("%s", func);
  switch (ack.info.cmd_set)
  {
    case OpenProtocol::CMDSet::activation:
      // CMD_ID agnostic
      getCMDSetActivationMSG(ack);
      break;
    case OpenProtocol::CMDSet::control:
      // Get message by CMD_ID
      getCMDSetControlMSG(ack);
    case OpenProtocol::CMDSet::broadcast:
      getSetBroadcastMSG(ack);
      break;
    case OpenProtocol::CMDSet::mission:
      // CMD_ID agnostic
      getCMDSetMissionMSG(ack);
      break;
    case OpenProtocol::CMDSet::hardwareSync:
      getCMDSetSyncMSG(ack);
      break;
    case OpenProtocol::CMDSet::virtualRC:
      getCMDSetVirtualRCMSG(ack);
      break;
    case OpenProtocol::CMDSet::mfio:
      getCMDSetMFIOMSG(ack);
      break;
    case OpenProtocol::CMDSet::subscribe:
      // CMD_ID agnostic
      getCMDSetSubscribeMSG(ack);
      break;
    default:
      DSTATUS("UNKNOWN_ACK_ERROR_CODE\n");
      break;
  }
}

/*
 * @note CMD_ID agnostic
 */
void
ACK::getCMDSetActivationMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> activateErrorCodeMap =
    ACK::createActivateErrorCodeMap();
  auto msg = activateErrorCodeMap.find(ack.data);

  if (msg != activateErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    getCommonErrorCodeMessage(ack);
  }
}

void
ACK::getCommonErrorCodeMessage(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> commonErrorCodeMap =
    ACK::createCommonErrorCodeMap();
  auto msg = commonErrorCodeMap.find(ack.data);

  if (msg != commonErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_ACK_ERROR_CODE\n");
  }
}

void
ACK::getCMDSetSubscribeMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> subscribeErrorCodeMap =
    ACK::createSubscribeErrorCodeMap();
  auto msg = subscribeErrorCodeMap.find(ack.data);

  if (msg != subscribeErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_SUBSCRIBER_ACK_ERROR_CODE_0x%X\n", ack.data);
  }
}

void
ACK::getCMDSetControlMSG(ACK::ErrorCode ack)
{
  const uint8_t cmd[] = { ack.info.cmd_set, ack.info.cmd_id };

  if (memcmp(cmd, OpenProtocol::CMDSet::Control::setControl, sizeof(cmd)) == 0)
  {
    getCMDIDSetControlMSG(ack.data, ack.info.version);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::control, sizeof(cmd)) ==
           0)
  {
    getCMDIDControlMSG(ack);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::task, sizeof(cmd)) == 0)
  {
    /*
     * @note Deprecated in 3.2.20
     */
    getCMDIDTaskMSG(ack);
  }
  else if (memcmp(cmd, OpenProtocol::CMDSet::Control::setArm, sizeof(cmd)) == 0)
  {
    /*
     * SetArm command supported in Matrice 100
     */
    getCMDIDSetArmMSG(ack);
  }
}

void
ACK::getCMDIDSetControlMSG(uint8_t ack, Version::FirmWare version)
{
  const std::map<const uint32_t, const char*> setControlErrorCodeMap =
    ACK::createSetControlErrorCodeMap();
  auto msg = setControlErrorCodeMap.find(ack);

  if (msg != setControlErrorCodeMap.end())
  {
    if (msg->first ==
        OpenProtocol::ErrorCode::ControlACK::SetControl::RC_MODE_ERROR)
    {
      if (version != Version::M100_31 || version != Version::A3_31)
      {
        DSTATUS("RC_NEED_MODE_P\n");
      }
      else
      {
        DSTATUS("RC_NEED_MODE_F\n");
      }
    }
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_ACK_ERROR_CODE\n");
  }
}

void
ACK::getCMDIDControlMSG(ACK::ErrorCode ack)
{
}

void
ACK::getCMDIDTaskMSG(ACK::ErrorCode ack)
{
  std::map<const uint32_t, const char*> taskErrorCodeMap;

  if (ack.info.version != Version::M100_31)
  {
    taskErrorCodeMap = ACK::createTaskErrorCodeMap();
  }
  else
  {
    taskErrorCodeMap = ACK::createM100TaskErrorCodeMap();
  }

  auto msg = taskErrorCodeMap.find(ack.data);

  if (msg != taskErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    getCommonErrorCodeMessage(ack);
  }
}

/*
 * SetArm command supported on Matrice 100 only
 */
void
ACK::getCMDIDSetArmMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> setArmErrorCodeMap =
    ACK::createSetArmErrorCodeMap();
  auto msg = setArmErrorCodeMap.find(ack.data);

  if (msg != setArmErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    getCommonErrorCodeMessage(ack);
  }
}

void
ACK::getSetBroadcastMSG(ACK::ErrorCode ack)
{
}

/*
 * @note CMD_ID agnostic
 *
 * @todo Check DJI_ERROR_CODE for comments
 */
void
ACK::getCMDSetMissionMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> missionErrorCodeMap =
    ACK::createMissionErrorCodeMap();
  auto msg = missionErrorCodeMap.find(ack.data);

  if (msg != missionErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("UNKNOWN_MISSION_ACK_ERROR_CODE\n");
  }
}

void
ACK::getCMDSetSyncMSG(ACK::ErrorCode ack)
{
}

void
ACK::getCMDSetVirtualRCMSG(ACK::ErrorCode ack)
{
}

void
ACK::getCMDSetMFIOMSG(ACK::ErrorCode ack)
{
  const std::map<const uint32_t, const char*> mfioErrorCodeMap =
    ACK::createMFIOErrorCodeMap();
  auto msg = mfioErrorCodeMap.find(ack.data);

  if (msg != mfioErrorCodeMap.end())
  {
    DSTATUS(msg->second);
  }
  else
  {
    DSTATUS("MFIO_UNKNOWN_ERROR\n");
  }
}

} // namespace OSDK
} // namespace DJI
