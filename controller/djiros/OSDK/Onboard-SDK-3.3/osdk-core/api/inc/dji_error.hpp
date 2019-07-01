/** @file dji_error.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK OpenProtocol ACK Error Codes
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJI_ERROR_H
#define DJI_ERROR_H

#include <stdint.h>

namespace DJI
{
namespace OSDK
{

/*! @class ErrorCode contains all the acknowledgments sent by the aircraft
 * @details Each component of the SDK has its own subclass defined within the
 * ErrorCode class.
 */
class ErrorCode
{
public:
  /*!
   * @brief Common ACK Error Codes
   */
  class CommonACK
  {
  public:
    const static uint16_t SUCCESS;
    const static uint16_t KEY_ERROR;
    const static uint16_t NO_AUTHORIZATION_ERROR;
    const static uint16_t NO_RIGHTS_ERROR;
    const static uint16_t NO_RESPONSE_ERROR;

    // These error codes would return either from
    // CMDSet Control::Task or from Missions
    const static uint8_t MOTOR_FAIL_NONE;
    /*! The compass being used appears as follows: <br>
     * (1) The compass data has too much noise. <br>
     * (2) The compass data is stuck. <br>
     * (3) The compass is disconnected. <br>
     * (4) Compass user compilation error. <br>
     * For the flight control of N3, A3 and M600, there are
     * more situations:  <br>
     * (5) The compass is disturbed.  <br>
     * (6) Multiple compasses point different directions. <br>
     * (7) Compass calibration failed. <br>
     * (8) The compass is not calibrated. */
    const static uint8_t MOTOR_FAIL_COMPASS_ABNORMAL;
    /*! The aircraft is connected to the software for debugging parameters via
       the
       USB cable. */
    const static uint8_t MOTOR_FAIL_ASSISTANT_PROTECTED;
    /*! The structure of the parameter list has changed after the FW upgrade.*/
    const static uint8_t MOTOR_FAIL_DEVICE_LOCKED;
    /*! The IMU being used appears as follows: <br>
     * (1) The accelerometer output exceeds its range. <br>
     * (2) The accelerometer is stuck. <br>
     * (3) The accelerometer data has too much noise. <br>
     * (4) The accelerometer outputs illegal floating numbers. <br>
     * (5) The factory data of IMU has exception. <br>
     * (6) Multiple accelerometers output differently. <br>
     * (7) The temperature of the IMU is too high. <br>
     * (8) The temperature of the IMU is very high. <br>
     * (9) The gyro output exceeds its range. <br>
     * (10) The gyro is stuck. <br>
     * (11) The gyro data has too much noise. <br>
     * (12) The gyro outputs illegal floating numbers. <br>
     * (13) Multiple accelerometers output differently. <br>
     * (14) The temperature control of gyro is abnormal. <br>
     * For the flight control of Inspire 2, there are more situations: <br>
     * (15)The default IMU exception causes the switch to backup IMU.*/
    const static uint8_t MOTOR_FAIL_IMU_NEED_ADV_CALIBRATION;
    /*! The SN status is wrong. */
    const static uint8_t MOTOR_FAIL_IMU_SN_ERROR;
    /*! The IMU being used is preheated and current temperature is not wihin the
       calibration range. */
    const static uint8_t MOTOR_FAIL_IMU_PREHEATING;
    const static uint8_t MOTOR_FAIL_COMPASS_CALIBRATING;
    const static uint8_t MOTOR_FAIL_IMU_NO_ATTITUDE;
    /*! The aircraft is in Novice Mode without gps. */
    const static uint8_t MOTOR_FAIL_NO_GPS_IN_NOVICE_MODE;
    const static uint8_t MOTOR_FAIL_BATTERY_CELL_ERROR;
    const static uint8_t MOTOR_FAIL_BATTERY_COMMUNICATION_ERROR;
    const static uint8_t MOTOR_FAIL_BATTERY_VOLTAGE_TOO_LOW;
    /*! The volume (%) is below the second-level power set by user. */
    const static uint8_t MOTOR_FAIL_BATTERY_USER_LOW_LAND;
    /*! The voltage is below the second-level power set by user. */
    const static uint8_t MOTOR_FAIL_BATTERY_MAIN_VOL_LOW;
    const static uint8_t MOTOR_FAIL_BATTERY_TEMP_VOL_LOW;
    /*! Flight contol calculates that current power is only adequate to land.*/
    const static uint8_t MOTOR_FAIL_BATTERY_SMART_LOW_LAND;
    /*! This error occurs whin 7s after power up.
     * Also, it occurs if the battery certification hasn't passed yet.*/
    const static uint8_t MOTOR_FAIL_BATTERY_NOT_READY;
    const static uint8_t MOTOR_FAIL_RUNNING_SIMULATOR;
    /*! The aircraft (Inspire series) is setting itself to packing config.*/
    const static uint8_t MOTOR_FAIL_PACK_MODE;
    const static uint8_t MOTOR_FAIL_IMU_ATTI_LIMIT;
    const static uint8_t MOTOR_FAIL_NOT_ACTIVATED;
    const static uint8_t MOTOR_FAIL_IN_FLYLIMIT_AREA;
    /*! The IMU is too biased if the gyro's bias is over 0.03rad/s
     * or the accelerometer's bias is over 50 mg when first started up.*/
    const static uint8_t MOTOR_FAIL_IMU_BIAS_LIMIT;
    const static uint8_t MOTOR_FAIL_ESC_ERROR;
    /*! The IMU is initializing.The attitude data of the current
     * navigation system has not converged yet and the height
     * data of the current navigation system is not ready.*/
    const static uint8_t MOTOR_FAIL_IMU_INITING;
    const static uint8_t MOTOR_FAIL_UPGRADING;
    /*! The simulator has already been run.*/
    const static uint8_t MOTOR_FAIL_HAVE_RUN_SIM;
    /*! The IMU is in calibration or the aircraft should reset after IMU
       calibration.*/
    const static uint8_t MOTOR_FAIL_IMU_CALIBRATING;
    const static uint8_t MOTOR_FAIL_TAKEOFF_TILT_TOO_LARGE;
    const static uint8_t MOTOR_FAIL_RESERVED_31;
    const static uint8_t MOTOR_FAIL_RESERVED_32;
    const static uint8_t MOTOR_FAIL_RESERVED_33;
    const static uint8_t MOTOR_FAIL_RESERVED_34;
    const static uint8_t MOTOR_FAIL_RESERVED_35;
    const static uint8_t MOTOR_FAIL_RESERVED_36;
    const static uint8_t MOTOR_FAIL_RESERVED_37;
    const static uint8_t MOTOR_FAIL_RESERVED_38;
    const static uint8_t MOTOR_FAIL_RESERVED_39;
    const static uint8_t MOTOR_FAIL_RESERVED_40;
    /*! invalid serial number */
    const static uint8_t MOTOR_FAIL_INVALID_SN;
    const static uint8_t MOTOR_FAIL_RESERVED_42;
    const static uint8_t MOTOR_FAIL_RESERVED_43;
    /*! accessing flash data, MCU is blocked */
    const static uint8_t MOTOR_FAIL_FLASH_OPERATING;
    const static uint8_t MOTOR_FAIL_GPS_DISCONNECT;
    const static uint8_t MOTOR_FAIL_INTERNAL_46;
    /*! SD card has an exception. Please repair SD card if repeats after
     * reset.*/
    const static uint8_t MOTOR_FAIL_RECORDER_ERROR;
    /*! The firmware is unmatched with configured type.*/
    const static uint8_t MOTOR_FAIL_INVALID_PRODUCT;
    const static uint8_t MOTOR_FAIL_RESERVED_49;
    const static uint8_t MOTOR_FAIL_RESERVED_50;
    const static uint8_t MOTOR_FAIL_RESERVED_51;
    const static uint8_t MOTOR_FAIL_RESERVED_52;
    const static uint8_t MOTOR_FAIL_RESERVED_53;
    const static uint8_t MOTOR_FAIL_RESERVED_54;
    const static uint8_t MOTOR_FAIL_RESERVED_55;
    const static uint8_t MOTOR_FAIL_RESERVED_56;
    const static uint8_t MOTOR_FAIL_RESERVED_57;
    const static uint8_t MOTOR_FAIL_RESERVED_58;
    const static uint8_t MOTOR_FAIL_RESERVED_59;
    const static uint8_t MOTOR_FAIL_RESERVED_60;
    const static uint8_t MOTOR_FAIL_IMU_DISCONNECTED;
    const static uint8_t MOTOR_FAIL_RC_CALIBRATING;
    const static uint8_t MOTOR_FAIL_RC_CALI_DATA_OUT_RANGE;
    const static uint8_t MOTOR_FAIL_RC_QUIT_CALI;
    const static uint8_t MOTOR_FAIL_RC_CENTER_OUT_RANGE;
    const static uint8_t MOTOR_FAIL_RC_MAP_ERROR;
    /*! The aircraft type in flash is unmatched with the type in firmware. <br>
     * Please check the aircraft type.*/
    const static uint8_t MOTOR_FAIL_WRONG_AIRCRAFT_TYPE;
    const static uint8_t MOTOR_FAIL_SOME_MODULE_NOT_CONFIGURED;
    const static uint8_t MOTOR_FAIL_RESERVED_69;
    const static uint8_t MOTOR_FAIL_RESERVED_70;
    const static uint8_t MOTOR_FAIL_RESERVED_71;
    const static uint8_t MOTOR_FAIL_RESERVED_72;
    const static uint8_t MOTOR_FAIL_RESERVED_73;
    /*! navigation system abnormal */
    const static uint8_t MOTOR_FAIL_NS_ABNORMAL;
    /*! Each craft has a set of devices to register. <br>
     * It won't take off if a class of device is missing. Please reset and check
     * the connection.*/
    const static uint8_t MOTOR_FAIL_TOPOLOGY_ABNORMAL;
    const static uint8_t MOTOR_FAIL_RC_NEED_CALI;
    /*! invalid data, system will block motor spinning */
    const static uint8_t MOTOR_FAIL_INVALID_FLOAT;
    const static uint8_t MOTOR_FAIL_M600_BAT_TOO_FEW;
    const static uint8_t MOTOR_FAIL_M600_BAT_AUTH_ERR;
    const static uint8_t MOTOR_FAIL_M600_BAT_COMM_ERR;
    /*! Battery voltage difference is too large. Please check the battery
       status.*/
    const static uint8_t MOTOR_FAIL_M600_BAT_DIF_VOLT_LARGE_1;
    const static uint8_t MOTOR_FAIL_BATTERY_BOLTAHGE_DIFF_82;
    const static uint8_t MOTOR_FAIL_INVALID_VERSION;
    /*! There is an gimbal attitude error which happens only in M600.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_GYRO_ABNORMAL;
    const static uint8_t MOTOR_FAIL_GIMBAL_ESC_PITCH_NO_DATA;
    const static uint8_t MOTOR_FAIL_GIMBAL_ESC_ROLL_NO_DATA;
    const static uint8_t MOTOR_FAIL_GIMBAL_ESC_YAW_NO_DATA;
    const static uint8_t MOTOR_FAIL_GIMBAL_FIRM_IS_UPDATING;
    const static uint8_t MOTOR_FAIL_GIMBAL_OUT_OF_CONTROL;
    /*! The gimbal has self-oscillation in the pitch direction. <br>
     * Please lock the camera or reduce the gimbal sensitivity.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_PITCH_SHOCK;
    /*! The gimbal has self-oscillation in the roll direction. <br>
     * Please lock the camera or reduce the gimbal sensitivity.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_ROLL_SHOCK;
    /*! The gimbal has self-oscillation in the yaw direction. <br>
     * Please lock the camera or reduce the gimbal sensitivity.*/
    const static uint8_t MOTOR_FAIL_GIMBAL_YAW_SHOCK;
    /*! IMU calibration finished. Please reset aircraft.*/
    const static uint8_t MOTOR_FAIL_IMU_CALI_SUCCESS;
    const static uint8_t MOTOR_FAIL_TAKEOFF_EXCEPTION;
    /*! The motor is locked. Please check the status of the motors and blades.*/
    const static uint8_t MOTOR_FAIL_ESC_STALL_NEAR_GOUND;
    /*! The feedback speed of motor is different with the input command.*/
    const static uint8_t MOTOR_FAIL_ESC_UNBALANCE_ON_GRD;
    /*! There are some no-load motors. Please check the status of the motors and
       blades.*/
    const static uint8_t MOTOR_FAIL_ESC_PART_EMPTY_ON_GRD;
    /*! During starting, the speed of any motor is less than the minimum
     * starting
     * speed. <br>
     * For N3 and A3, the minimum starting speed is 100rpm. <br>
     * For M600, the minimum starting speed is 700rpm. <br>
     * For other aircrafts, the minimum starting speed is 1100rpm.*/
    const static uint8_t MOTOR_FAIL_ENGINE_START_FAILED;
    const static uint8_t MOTOR_FAIL_AUTO_TAKEOFF_LAUNCH_FAILED;
    const static uint8_t MOTOR_FAIL_ROLL_OVER_ON_GRD;
    const static uint8_t MOTOR_FAIL_BAT_VERSION_ERR;
    const static uint8_t MOTOR_FAIL_RTK_INITING;
    /*! rtk yaw and magnetometer yaw misaligned */
    const static uint8_t MOTOR_FAIL_RTK_FAIL_TO_INIT;
    const static uint8_t MOTOR_FAIL_RESERVED_104;
    const static uint8_t MOTOR_FAIL_RESERVED_105;
    const static uint8_t MOTOR_FAIL_RESERVED_106;
    const static uint8_t MOTOR_FAIL_RESERVED_107;
    const static uint8_t MOTOR_FAIL_RESERVED_108;
    const static uint8_t MOTOR_FAIL_RESERVED_109;
    /*! The motor status shows the motor has been started.*/
    const static uint8_t START_MOTOR_FAIL_MOTOR_STARTED;
    const static uint8_t MOTOR_FAIL_INTERNAL_111;
    const static uint8_t MOTOR_FAIL_ESC_CALIBRATING;
    const static uint8_t MOTOR_FAIL_GPS_SIGNATURE_INVALID;
    const static uint8_t MOTOR_FAIL_GIMBAL_CALIBRATING;
    /*! The aircraft is force locked by APP.*/
    const static uint8_t MOTOR_FAIL_FORCE_DISABLE;
    /*! The height of the takeoff is abnormal. <br>
     * This error happens when the takeoff height relative to ground is up to
     * 100m.*/
    const static uint8_t TAKEOFF_HEIGHT_EXCEPTION;
    const static uint8_t MOTOR_FAIL_ESC_NEED_UPGRADE;
    /*! IMU direction is misaligned.*/
    const static uint8_t MOTOR_FAIL_GYRO_DATA_NOT_MATCH;
    /*! APP stops the takeoff.*/
    const static uint8_t MOTOR_FAIL_APP_NOT_ALLOW;
    const static uint8_t MOTOR_FAIL_COMPASS_IMU_MISALIGN;
    const static uint8_t MOTOR_FAIL_FLASH_UNLOCK;
    /*! The ESC is in the buzzing mode.*/
    const static uint8_t MOTOR_FAIL_ESC_SCREAMING;
    const static uint8_t MOTOR_FAIL_ESC_TEMP_HIGH;
    /*! The battery is not in place. */
    const static uint8_t MOTOR_FAIL_BAT_ERR;
    /*! The aircraft detects an impact if the measured value of
     * accelerometer exceeds 8g near ground.*/
    const static uint8_t IMPACT_IS_DETECTED;
    /*! Under the P stall, the aircraft mode degenerates to the Attitude mode.*/
    const static uint8_t MOTOR_FAIL_MODE_FAILURE;
    /*! The aircraft recently had an error of NO. 125.*/
    const static uint8_t MOTOR_FAIL_CRAFT_FAIL_LATELY;
    /*! The code logic is illegal.*/
    const static uint8_t MOTOR_FAIL_MOTOR_CODE_ERROR;
  };

  /*!
   * @brief CMDSet: Activation ACK Error Codes
   */
  class ActivationACK
  {
  public:
    const static uint16_t SUCCESS;
    const static uint16_t PARAMETER_ERROR;
    const static uint16_t ENCODE_ERROR;
    const static uint16_t NEW_DEVICE_ERROR;
    const static uint16_t DJIGO_APP_NOT_CONNECTED;
    const static uint16_t NETWORK_ERROR;
    const static uint16_t SERVER_ACCESS_REFUSED;
    const static uint16_t ACCESS_LEVEL_ERROR;
    const static uint16_t OSDK_VERSION_ERROR;
  };

  /*!
   * @brief CMDSet: Control ACK Error Codes
   */
  class ControlACK
  {
  public:
    /*!
     * @brief CMDID: SetControl
     */
    typedef struct SetControl
    {
      const static uint16_t RC_MODE_ERROR;
      const static uint16_t RELEASE_CONTROL_SUCCESS;
      const static uint16_t OBTAIN_CONTROL_SUCCESS;
      const static uint16_t OBTAIN_CONTROL_IN_PROGRESS;
      const static uint16_t RELEASE_CONTROL_IN_PROGRESS;
      const static uint16_t RC_NEED_MODE_F;
      const static uint16_t RC_NEED_MODE_P;
      const static uint16_t IOC_OBTAIN_CONTROL_ERROR;
    } SetControl;

    /*!
     * @note New 3.3 release
     *
     * @brief CMDID: Task
     */
    typedef struct Task
    {
      const static uint16_t SUCCESS;
      const static uint16_t MOTOR_ON;
      const static uint16_t MOTOR_OFF;
      const static uint16_t IN_AIR;
      const static uint16_t NOT_IN_AIR;
      const static uint16_t NO_HOMEPOINT;
      const static uint16_t BAD_GPS;
      // Do not consider as error?
      const static uint16_t IN_SIMULATOR_MODE;
      const static uint16_t ALREADY_RUNNING;
      const static uint16_t NOT_RUNNING;
      const static uint16_t INVAILD_COMMAND;
      const static uint16_t NO_LANDING_GEAR;
      // Do not consider as error?
      const static uint16_t GIMBAL_MOUNTED;
      const static uint16_t BAD_SENSOR;
      const static uint16_t ALREADY_PACKED;
      const static uint16_t NO_PACKED;
      const static uint16_t PACKED_MODE_NOT_SUPPORTED;
    } Task;

    /*
     * Task ACKs Supported in firmware version < 3.3
     */
    typedef struct M100Task
    {
      const static uint16_t SUCCESS;
      const static uint16_t FAIL;
    } M100Task;

    /*!
     * @brief CMDID: SetArm supported in products with
     * firmware version < 3.3
     */
    typedef struct SetArm
    {
      const static uint16_t SUCCESS;
      const static uint16_t OBTAIN_CONTROL_NEEDED_ERROR;
      const static uint16_t ALREADY_ARMED_ERROR;
      const static uint16_t AIRCRAFT_IN_AIR_ERROR;
    } SetArm;

  }; // Control class

  /*!
   * @note New in 3.3 release
   *
   * @brief CMDSet: Subscribe
   */
  class SubscribeACK
  {
  public:
    const static uint8_t SUCCESS;
    const static uint8_t ILLEGAL_INPUT;
    const static uint8_t VERSION_DOES_NOT_MATCH;
    const static uint8_t PACKAGE_OUT_OF_RANGE;
    const static uint8_t PACKAGE_ALREADY_EXISTS;
    const static uint8_t PACKAGE_DOES_NOT_EXIST;
    const static uint8_t ILLEGAL_FREQUENCY;
    const static uint8_t PACKAGE_TOO_LARGE;
    const static uint8_t PIPELINE_OVERFLOW;
    const static uint8_t INTERNAL_ERROR_0X09;
    const static uint8_t PACKAGE_EMPTY;
    const static uint8_t INPUT_SEGMENTATION_FAULT;
    const static uint8_t ILLEGAL_UID;
    const static uint8_t PERMISSION_DENY;
    const static uint8_t MULTIPLE_SUBSCRIBE;
    const static uint8_t SOUCE_DEVICE_OFFLINE;
    const static uint8_t PAUSED;
    const static uint8_t RESUMED;
    const static uint8_t INTERNAL_ERROR_0X4A;
    const static uint8_t INTERNAL_ERROR_0X50;
    const static uint8_t VERSION_VERSION_TOO_FAR;
    const static uint8_t VERSION_UNKNOWN_ERROR;
    const static uint8_t INTERNAL_ERROR_0XFF;
  };

  /*!
   * @brief Mission ACK Error Codes
   */
  class MissionACK
  {
  public:
    /*! @brief Common Mission ACK codes
     *
     */
    typedef struct Common
    {
      const static uint8_t SUCCESS;
      const static uint8_t WRONG_WAYPOINT_INDEX;
      const static uint8_t RC_NOT_IN_MODE_F;
      const static uint8_t OBTAIN_CONTROL_REQUIRED;
      const static uint8_t CLOSE_IOC_REQUIRED;
      const static uint8_t NOT_INITIALIZED;
      const static uint8_t NOT_RUNNING;
      const static uint8_t IN_PROGRESS;
      /*!Estimated time needed to perform a task is greater
       * than the flight time left*/
      const static uint8_t TASK_TIMEOUT;
      const static uint8_t OTHER_MISSION_RUNNING;
      /*!GPS signal GPS_LEVEL < 3*/
      const static uint8_t BAD_GPS;
      const static uint8_t RTK_NOT_READY;
      /*!Battery beyond first-stage voltage for non-smart battery
       * OR first-stage volume for smart battery*/
      const static uint8_t LOW_BATTERY;
      const static uint8_t VEHICLE_DID_NOT_TAKE_OFF;
      const static uint8_t INVALID_PARAMETER;
      /*!Execution condition is not satisfied
       * @note Aircraft not in one of the following modes:
       * Assist Takeoff
       * Auto Takeoff
       * Auto Landing
       * Go Home
       */
      const static uint8_t CONDITIONS_NOT_SATISFIED;
      const static uint8_t CROSSING_NO_FLY_ZONE;
      /*!HomePoint not recorded*/
      const static uint8_t UNRECORDED_HOME;
      const static uint8_t AT_NO_FLY_ZONE;
      /*!Height is too high (higher than MAX flying height
       * set by user (default: 120m))*/
      const static uint8_t TOO_HIGH;
      /*!Height is too low (lower than 5m)*/
      const static uint8_t TOO_LOW;
      const static uint8_t TOO_FAR_FROM_HOME;
      /*!Mission not supported*/
      const static uint8_t NOT_SUPPORTED;
      /*!Current position of aircraft is too far from the HotPoint
       * or first point*/
      const static uint8_t TOO_FAR_FROM_CURRENT_POSITION;
      const static uint8_t BEGGINER_MODE_NOT_SUPPORTED;
      const static uint8_t TAKE_OFF_IN_PROGRESS;
      const static uint8_t LANDING_IN_PROGRESS;
      const static uint8_t RRETURN_HOME_IN_PROGRESS;
      const static uint8_t START_MOTORS_IN_PROGRESS;
      const static uint8_t INVALID_COMMAND;
      const static uint8_t UNKNOWN_ERROR;
    } Common;

    //! @brief Follow Mission ACK Error Code
    typedef struct Follow
    {
      const static uint8_t TOO_FAR_FROM_YOUR_POSITION_LACK_OF_RADIO_CONNECTION;
      const static uint8_t CUTOFF_TIME_OVERFLOW;
      const static uint8_t GIMBAL_PITCH_ANGLE_OVERFLOW;
    } Follow;

    //! @brief HotPoint Mission ACK Error Code
    typedef struct HotPoint
    {
      const static uint8_t INVALID_RADIUS;
      const static uint8_t YAW_RATE_OVERFLOW;
      /*
       * Start point given by user during HotPoint mission initialization is
       * invalid.
       * Available options are :
       * 0 - North to the HP
       * 1 - South
       * 2 - West
       * 3 - East
       * 4 - Nearest Point
       */
      const static uint8_t INVALID_START_POINT;
      const static uint8_t INVALID_YAW_MODE;
      const static uint8_t TOO_FAR_FROM_HOTPOINT;
      const static uint8_t INVALID_PAREMETER;
      const static uint8_t INVALID_LATITUDE_OR_LONGITUTE;
      const static uint8_t INVALID_DIRECTION;
      const static uint8_t IN_PAUSED_MODE;
      const static uint8_t FAILED_TO_PAUSE;
    } HotPoint;

    //! @brief WayPoint Mission ACK Error Code
    typedef struct WayPoint
    {
      const static uint8_t INVALID_DATA;
      const static uint8_t INVALID_POINT_DATA;
      const static uint8_t DISTANCE_OVERFLOW;
      const static uint8_t TIMEOUT;
      const static uint8_t POINT_OVERFLOW;
      const static uint8_t POINTS_TOO_CLOSE;
      const static uint8_t POINTS_TOO_FAR;
      const static uint8_t CHECK_FAILED;
      const static uint8_t INVALID_ACTION;
      const static uint8_t POINT_DATA_NOT_ENOUGH;
      const static uint8_t DATA_NOT_ENOUGH;
      const static uint8_t POINTS_NOT_ENOUGH;
      const static uint8_t IN_PROGRESS;
      const static uint8_t NOT_IN_PROGRESS;
      const static uint8_t INVALID_VELOCITY;
    } WayPoint;

    //! @brief IOC ACK Mission Error Code
    typedef struct IOC
    {
      const static uint8_t TOO_CLOSE_TO_HOME;
      const static uint8_t UNKNOWN_TYPE;
    } IOC;

  }; // Class Mission

  /*!
   * @brief CMDSet: MFIO
   * @note New in 3.3 release
   */
  class MFIOACK
  {
  public:
    /*!
     * @brief CMDID: init
     */
    typedef struct init
    {
      const static uint8_t SUCCESS;
      const static uint8_t UNKNOWN_ERROR;
      const static uint8_t PORT_NUMBER_ERROR;
      const static uint8_t PORT_MODE_ERROR;
      const static uint8_t PORT_DATA_ERROR;
    } init;

    /*!
     * @brief CMDID: set
     */
    typedef struct set
    {
      const static uint8_t SUCCESS;
      /*!Port not exit or not an output configuration*/
      const static uint8_t CHANNEL_ERROR;
      /*! Port not map to f channel*/
      const static uint8_t PORT_NOT_MAPPED_ERROR;
    } set;

    /*!
     * @brief CMDID: get
     */
    typedef struct get
    {
      const static uint8_t SUCCESS; //! @note Anything else is failure
    } get;

  }; // Class MFIO

}; // Class ErrorCode

} // namespace OSDK
} // namespace DJI

#endif /* DJI_ERROR_H */
