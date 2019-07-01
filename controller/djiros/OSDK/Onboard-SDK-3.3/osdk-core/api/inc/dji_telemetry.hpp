/** @file dji_telemetry.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Enumeration of all telemetry data types, structures and maps.
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef ONBOARDSDK_DJI_Telemetry_H
#define ONBOARDSDK_DJI_Telemetry_H

#include "dji_type.hpp"

/*!
 * Top-level namespace
 */
namespace DJI
{
/*!
 * Onboard SDK related commands
 */
namespace OSDK
{
/*! @brief This namespace encapsulates all available telemetry topics through
 * either
 * Broadcast or Subscribe
 */
namespace Telemetry
{
/*!
 * @brief enum TopicName is the interface for user to create packages and access
 * data
 * It is also used as index for TopicDataBase
 *
 */
// clang-format off
typedef enum
{
  TOPIC_QUATERNION,               /*!< quaternion @200Hz*/
  TOPIC_ACCELERATION_GROUND,      /*!< acceleration in ground frame @200Hz */
  TOPIC_ACCELERATION_BODY,        /*!< acceleration in body frame @200Hz*/
  TOPIC_ACCELERATION_RAW,         /*!< raw acceleration @400Hz */
  TOPIC_VELOCITY,                 /*!< velocity @200Hz */
  TOPIC_ANGULAR_RATE_FUSIONED,    /*!< attitude rate @200Hz */
  TOPIC_ANGULAR_RATE_RAW,         /*!< raw attitude rate @400Hz */
  TOPIC_ALTITUDE_FUSIONED,        /*!< fused altitude @200Hz */
  TOPIC_ALTITUDE_BAROMETER,       /*!< barometer @200Hz */
  TOPIC_HEIGHT_HOMEPOINT,         /*!< home point height @1Hz */
  TOPIC_HEIGHT_FUSION,            /*!< fused height @100Hz */
  TOPIC_GPS_FUSED,                /*!< fused position @50Hz */
  TOPIC_GPS_DATE,                 /*!< GPS date @50Hz */
  TOPIC_GPS_TIME,                 /*!< GPS time @50Hz */
  TOPIC_GPS_POSITION,             /*!< GPS position data type: (uint32)deg*10^7 @50Hz */
  TOPIC_GPS_VELOCITY,             /*!< GPS velocity @50Hz */
  TOPIC_GPS_DETAILS,              /*!< GPS status @50Hz */
  TOPIC_RTK_POSITION,             /*!< RTK position @50Hz */
  TOPIC_RTK_VELOCITY,             /*!< RTK velocity @50Hz */
  TOPIC_RTK_YAW,                  /*!< RTK yaw @50Hz */
  TOPIC_RTK_POSITION_INFO,        /*!< RTK status @50Hz */
  TOPIC_RTK_YAW_INFO,             /*!< RTK yaw status @50Hz */
  TOPIC_COMPASS,                  /*!< magnetometer @100Hz */
  TOPIC_RC,                       /*!< 6-channel RC @50Hz */
  TOPIC_GIMBAL_ANGLES,            /*!< gimbal angle @50Hz */
  TOPIC_GIMBAL_STATUS,            /*!< gimbal status @50Hz */
  TOPIC_STATUS_FLIGHT,            /*!< flight status @50Hz */
  TOPIC_STATUS_DISPLAYMODE,       /*!< display mode @50Hz */
  TOPIC_STATUS_LANDINGGEAR,       /*!< landing gear status @50Hz */
  TOPIC_STATUS_MOTOR_START_ERROR, /*!< motor start error @50Hz */
  TOPIC_BATTERY_INFO,             /*!< battery info @50Hz */
  TOPIC_CONTROL_DEVICE,           /*!< SDK control device info @50Hz */
  TOPIC_HARD_SYNC,                /*!< hardware sync @400Hz */
  TOPIC_GPS_SIGNAL_LEVEL,         /*!< GPS Signal Level @50Hz */
  TOPIC_GPS_CONTROL_LEVEL,        /*!< GPS Control Level @50Hz */
  TOTAL_TOPIC_NUMBER              // Always put this line in the end
} TopicName;
// clang-format on

/*!
 * enum TOPIC_UID is the UID that is accepted by the FC
 */
// clang-format off
typedef enum
{
  UID_QUATERNION               = 0xa493281f,
  UID_ACCELERATION_GROUND      = 0x8696c85b,
  UID_ACCELERATION_BODY        = 0xbb17d5fe,
  UID_ACCELERATION_RAW         = 0xc3503a6e,
  UID_VELOCITY                 = 0x18fb271d,
  UID_ANGULAR_RATE_FUSIONED    = 0x3599c4be,
  UID_ANGULAR_RATE_RAW         = 0x700389ee,
  UID_ALTITUDE_FUSIONED        = 0x11e9c81a,
  UID_ALTITUDE_BAROMETER       = 0x27396a39,
  UID_HEIGHT_HOMEPOINT         = 0x252c164b,
  UID_HEIGHT_FUSION            = 0x87cf419d,
  UID_GPS_FUSED                = 0x4b19a8c7,
  UID_GPS_DATE                 = 0x598f79bc,
  UID_GPS_TIME                 = 0xd48912c9,
  UID_GPS_POSITION             = 0x0c949e94,
  UID_GPS_VELOCITY             = 0x7ac7eb80,
  UID_GPS_DETAILS              = 0x81fed54e,
  UID_RTK_POSITION             = 0x1df9a6b6,
  UID_RTK_VELOCITY             = 0x763d13c3,
  UID_RTK_YAW                  = 0xf45d73fd,
  UID_RTK_POSITION_INFO        = 0xda4a57b5,
  UID_RTK_YAW_INFO             = 0xcb72b9e3,
  UID_COMPASS                  = 0xdf3d72b7,
  UID_RC                       = 0x739f7fe4,
  UID_GIMBAL_ANGLES            = 0x01f71678,
  UID_GIMBAL_STATUS            = 0x8b6cd45c,
  UID_STATUS_FLIGHT            = 0x20cfb02a,
  UID_STATUS_DISPLAYMODE       = 0x1a67d6a1,
  UID_STATUS_LANDINGGEAR       = 0x772d6e22,
  UID_STATUS_MOTOR_START_ERROR = 0x3a41e909,
  UID_BATTERY_INFO             = 0x69779dd9,
  UID_CONTROL_DEVICE           = 0x667ba86a,
  UID_HARD_SYNC                = 0xecbef06d,
  UID_GPS_SIGNAL_LEVEL         = 0xa6a0395f,
  UID_GPS_CONTROL_LEVEL        = 0xe30b17b0
} TOPIC_UID;
// clang-format on

#pragma pack(1)
typedef struct
{
  const TopicName name;
  const uint32_t  uid;
  const size_t    size;    /* The size of actual data for the topic */
  const uint16_t  maxFreq; /* max freq in Hz for the topic provided by FC */
  uint16_t        freq;    /* Frequency at which the topic is subscribed */
  uint8_t         pkgID;   /* Package ID in which the topic is subscribed */
  /* Point to topic's address in the data buffer which stores the latest data */
  uint8_t* latest;
} TopicInfo; // pack(1)

/*! @brief struct for TOPIC_QUATERNION
 *
 */
typedef struct Quaternion
{
  float32_t q0; /*!< w */
  float32_t q1; /*!< x */
  float32_t q2; /*!< y */
  float32_t q3; /*!< z */
} Quaternion;   // pack(1)

/*!
 * @brief struct for multiple Topics
 */
typedef struct Vector3f
{
  float32_t x;
  float32_t y;
  float32_t z;
} Vector3f; // pack(1)

/*!
 * @brief struct for multiple Topics
 *
 * @note for TOPIC_GPS_POSITION, data type: (uint32)deg*10^7
 */
typedef struct Vector3d
{
  int32_t x;
  int32_t y;
  int32_t z;
} Vector3d; // pack(1)

/*!
 * @brief struct for data broadcast, timestamp from local cache
 *
 * @note not available in data subscription
 */
typedef struct TimeStamp
{
  uint32_t time_ms;
  uint32_t time_ns;
} TimeStamp; // pack(1)

/*!
 * @brief struct for data broadcast, software sync timestamp from local cache
 *
 * @note not available in data subscription and different from Hardware sync
 */
typedef struct SyncStamp
{
  uint32_t time_2p5ms; /*!< relative sync time */
  uint16_t tag;
  uint8_t  flag;
} SyncStamp; // pack(1)

/*!
 * @brief struct indicates the signal level of GPS velocity info <br>
 *
 */
typedef struct VelocityInfo
{

  uint8_t health : 1; /*!< 1 - using GPS, 0 - not using GPS */
  uint8_t reserve : 7;
} VelocityInfo; // pack(1)

/*!
 * @brief struct for TOPIC_VELOCITY
 *
 * @note The velocity may be in body or ground frame
 * based on settings in DJI Assistant 2's SDK page.
 */
typedef struct Velocity
{
  Vector3f data;
  /*! scale from 0 - 5 signifying gps signal strength <br>
   *  greater than 3 for strong signal
   */
  VelocityInfo info;
} Velocity; // pack(1)

/*!
 * @brief struct for data broadcast, return GPS data
 *
 * @note not available in data subscription
 */
typedef struct GlobalPosition
{
  float64_t latitude;  /*!< unit: rad */
  float64_t longitude; /*!< unit: rad */
  float32_t altitude;  /*!< WGS 84 reference ellipsoid */
  float32_t height;    /*!< relative height to the ground */
  uint8_t   health;    /*!< scale from 0 - 5 signifying gps signal strength <br>
                        * greater than 3 for strong signal */
} GlobalPosition;      // pack(1)

/*!
 * @brief struct for TOPIC_GPS_FUSED
 *
 * @note fusion data from GPS and IMU, return in gps format
 */
typedef struct GPSFused
{
  float64_t longitude;              /*!< unit: rad */
  float64_t latitude;               /*!< unit: rad */
  float32_t altitude;               /*!< WGS 84 reference ellipsoid */
  uint16_t  visibleSatelliteNumber; /*!< number of visible satellite */
} GPSFused;                         // pack(1)

/*!
 * @brief struct for data broadcast, return obstacle info around the vehicle
 *
 * @note available in M210 (front, up, down)
 */
typedef struct RelativePosition
{
  float32_t down;            /*!< distance from obstacle (cm) */
  float32_t front;           /*!< distance from obstacle (cm) */
  float32_t right;           /*!< distance from obstacle (cm) */
  float32_t back;            /*!< distance from obstacle (cm) */
  float32_t left;            /*!< distance from obstacle (cm) */
  float32_t up;              /*!< distance from obstacle (cm) */
  uint8_t   downHealth : 1;  /*!< 0 - not working, 1 - working */
  uint8_t   frontHealth : 1; /*!< 0 - not working, 1 - working */
  uint8_t   rightHealth : 1; /*!< 0 - not working, 1 - working */
  uint8_t   backHealth : 1;  /*!< 0 - not working, 1 - working */
  uint8_t   leftHealth : 1;  /*!< 0 - not working, 1 - working */
  uint8_t   upHealth : 1;    /*!< 0 - not working, 1 - working */
  uint8_t   reserved : 2;
} RelativePosition; // pack(1)

/*!
 * @brief sub struct for GPSInfo
 */
typedef struct PositionTimeStamp
{
  uint32_t date;     /*!< yyyy-mm-dd */
  uint32_t time;     /*!< hh-mm-ss */
} PositionTimeStamp; // pack(1)

/*!
 * @brief struct for TOPIC_RTK_POSITION and sub struct for RTK of data broadcast
 */
typedef struct PositionData
{
  float64_t longitude; /*!< deg */
  float64_t latitude;  /*!< deg */
  float32_t HFSL;      /*!< height above mean sea level (m) */
} PositionData;        // pack(1)

/*!
 * @brief struct for TOPIC_GPS_DETAILS and sub struct for GPSInfo of data
 * broadcast
 *
 * @note only work outside of simulation
 */
typedef struct GPSDetail
{
  float32_t hdop;       /*!< horizontal dilution of precision */
  float32_t pdop;       /*!< position dilution of precision */
  float32_t fix;        /*!< the state of GPS fix */
  float32_t gnssStatus; /*!< vertical position accuracy (mm) */
  float32_t hacc;       /*!< horizontal position accuracy (mm) */
  float32_t sacc;       /*!< the speed accuracy (cm/s) */
  uint32_t  usedGPS;    /*!< the number of GPS satellites used for pos fix */
  uint32_t  usedGLN; /*!< the number of GLONASS satellites used for pos fix */
  uint16_t  NSV;     /*!< the total number of satellites used for pos fix */
  uint16_t  GPScounter; /*!< the accumulated times of sending GPS data  */
} GPSDetail;            // pack(1)

/*!
 * @brief struct for GPSInfo of data broadcast
 *
 * @note only work outside of simulation
 */
typedef struct GPSInfo
{
  PositionTimeStamp time;
  int32_t           longitude;   /*!< 1/1.0e7deg */
  int32_t           latitude;    /*!< 1/1.0e7deg */
  int32_t           HFSL;        /*!< height above mean sea level (mm) */
  Vector3f          velocityNED; /*!< cm/s */
  GPSDetail         detail;
} GPSInfo; // pack(1)

/*!
 * @brief sub struct for RTK of data broadcast
 */
typedef struct PositionFrame
{
  PositionTimeStamp time;
  PositionData      data;
} PositionFrame; // pack(1)

/*!
 * @brief struct for data broadcast, return RTK info
 *
 * @note Available on A3/M600, need to enable it separately on DJI Assistant 2
 */
typedef struct RTK
{
  PositionFrame pos;
  Vector3f      velocityNED;
  /*! the azimuth measured by RTK */
  int16_t yaw;
  /*!
   * 0 - no solution <br>
   * 1 - Position has been fixed by the FIX POSITION command  <br>
   * 2 - Position has been fixed by the FIX HEIGHT/AUTO command <br>
   * 8 - Velocity computed using instantaneous Doppler <br>
   * 16 - Single point position <br>
   * 17 - Pseudorange differential solution <br>
   * 18 - Solution calculated using corrections from an SBAS <br>
   * 19 - Propagated by a Kalman filter without new observations <br>
   * 20 - OmniSTAR VBS position (L1 sub-metre) <br>
   * 32 - Floating L1 ambiguity solution <br>
   * 33 - Floating ionospheric-free ambiguity solution <br>
   * 34 - Floating narrow-lane ambiguity solution <br>
   * 48 - Integer L1 ambiguity solution <br>
   * 49 - Integer wide-lane ambiguity solution <br>
   * 50 - Integer narrow-lane ambiguity solution <br>
   */
  uint8_t posHealthFlag;
  uint8_t yawHealthFlag; /*!< same as posHealthFlag */
} RTK;                   // pack(1)

/*!
 * @brief struct for data broadcast, return magnetometer reading
 *
 * @note returned value is calibrated mag data,
 * 1000 < |mag| < 2000 for normal operation
 */
typedef struct Mag
{
  int16_t x;
  int16_t y;
  int16_t z;
} Mag; // pack(1)

/*!
 * @brief struct for data broadcast and data subscription, return RC reading
 */
typedef struct RC
{
  int16_t roll;     /*!< [-10000,10000] */
  int16_t pitch;    /*!< [-10000,10000] */
  int16_t yaw;      /*!< [-10000,10000] */
  int16_t throttle; /*!< [-10000,10000] */
  int16_t mode;     /*!< P: -8000, A: 0, F: 8000 */
  int16_t gear;     /*!< Up: -10000, Down: -4545 */
} RC;               // pack(1)

/*!
 * @brief struct for TOPIC_GIMBAL_STATUS
 */
typedef struct GimbalStatus
{
  uint32_t mountStatus : 1;            /*!< 1 - gimbal mounted, 0 - gimbal not mounted*/
  uint32_t isBusy : 1;
  uint32_t pitchLimited : 1;           /*!< 1 - axis reached limit, 0 - no */
  uint32_t rollLimited : 1;            /*!< 1 - axis reached limit, 0 - no */
  uint32_t yawLimited : 1;             /*!< 1 - axis reached limit, 0 - no */
  uint32_t calibrating : 1;            /*!< 1 - calibrating, 0 - no */
  uint32_t prevCalibrationgResult : 1; /*!< 1 - success, 0 - fail */
  uint32_t installedDirection : 1;     /*!< 1 - reversed for OSMO, 0 - normal */
  uint32_t disabled_mvo : 1;
  uint32_t gear_show_unable : 1;
  uint32_t gyroFalut : 1;      /*!< 1 - at fault, 0 - normal */
  uint32_t escPitchFault : 1;  /*!< 1 - at fault, 0 - normal */
  uint32_t escRollFault : 1;   /*!< 1 - at fault, 0 - normal */
  uint32_t escYawFault : 1;    /*!< 1 - at fault, 0 - normal */
  uint32_t droneDataFault : 1; /*!< 1 - at fault, 0 - normal */
  uint32_t initUnfinished : 1; /*!< 1 - complete, 0 - not complete */
  uint32_t FWUpdating : 1;     /*!< 1 - updating, 0 - not updating */
  uint32_t reserved2 : 15;
} GimbalStatus; // pack(1)

/*!
 * @brief struct for data broadcast, return gimbal angle
 */
typedef struct Gimbal
{

  float32_t roll;           /*!< degree */
  float32_t pitch;          /*!< degree */
  float32_t yaw;            /*!< degree */
  uint8_t   pitchLimit : 1; /*!< 1 - axis reached limit, 0 - no */
  uint8_t   rollLimit : 1;  /*!< 1 - axis reached limit, 0 - no */
  uint8_t   yawLimit : 1;   /*!< 1 - axis reached limit, 0 - no */
  uint8_t   reserved : 5;
} Gimbal; // pack(1)

/*!
 * @brief struct for data broadcast, return flight status
 */
typedef struct Status
{
  uint8_t flight; /*!<  enum STATUS */
  uint8_t mode;   /*!<  enum MODE */
  uint8_t gear;   /*!<  enum LANDING_GEAR */
  uint8_t error;  /*!<  enum DJI_ERROR_CODE */
} Status;         // pack(1)

/*!
 * @brief struct for TOPIC_BATTERY_INFO and data broadcast, return battery
 * status
 */
typedef struct Battery
{
  uint32_t capacity;
  int32_t  voltage;
  int32_t  current;
  uint8_t  percentage;
} Battery; // pack(1)

/*!
 * @brief struct for TOPIC_CONTROL_DEVICE and data broadcast, return SDK info
 */
typedef struct SDKInfo
{
  uint8_t controlMode;      /*!< enum CTRL_MODE */
  uint8_t deviceStatus : 3; /*!< 0->rc  1->app  2->serial*/
  uint8_t flightStatus : 1; /*!< 1->opensd  0->close */
  uint8_t vrcStatus : 1;
  uint8_t reserved : 3;
} SDKInfo; // pack(1)

/*!
 * @brief sub struct for TOPIC_HARD_SYNC
 */
typedef struct SyncTimestamp
{
  uint32_t time2p5ms; /*!< clock time in multiples of 2.5ms. Sync timer runs at
                         400Hz, this field increments in integer steps */
  uint32_t time1ns;   /*!< nanosecond time offset from the 2.5ms pulse */
  uint32_t
    resetTime2p5ms; /*!< clock time in multiple of 2.5ms elapsed since the
                hardware sync started */
  uint16_t index;   /*!< This is the tag field you filled out when using the
                       setSyncFreq API above; use it to identify the packets that
                       have sync data. This is useful when you call the
                       setSyncFreq API with freqInHz = 0, so you get a single
                       pulse that can be uniquely identified with a tag - allowing
                       you to create your own pulse train with uniquely
                       identifiable pulses. */
  uint8_t flag;     /*!< This is true when the packet corresponds to a hardware
                       pulse and false otherwise. This is useful because you can
                       request the software packet to be sent at a higher frequency
                       that the hardware line.*/
} SyncTimestamp;    // pack(1)

/*!
 * @brief struct for TOPIC_HARD_SYNC
 */
typedef struct HardSyncData
{
  SyncTimestamp ts; /*!< time stamp for the incoming data */
  Quaternion    q;  /*!< quaternion */
  Vector3f      a;  /*!< accelerometer reading unit: g */
  Vector3f      w;  /*!< gyro reading unit: rad/sec */
} HardSyncData;     // pack(1)

/*!
 * @brief Matrice 100 Timestamp data, available in Broadcast telemetry (only for M100)
 */
typedef struct M100TimeStamp
{
  uint32_t time;
  uint32_t nanoTime;
  uint8_t  syncFlag;
} M100TimeStamp; // pack(1)

/*!
 * @brief Matrice 100 Velocity struct, returned in Broadcast telemetry (only for M100)
 * @note The velocity may be in body or ground frame
 * based on settings in DJI Assistant 2's SDK page.
 */
typedef struct M100Velocity
{
  float32_t x;
  float32_t y;
  float32_t z;
  /*! scale from 0 - 5 signifying gps signal strength <br>
   *  greater than 3 for strong signal
   */
  uint8_t health : 1;
  uint8_t sensorID : 4;
  uint8_t reserve : 3;
} M100Velocity; // pack(1)

typedef uint16_t EnableFlag;  // pack(1)

/*!
 * @brief Return type for flight status data broadcast (only for M100). Returns VehicleStatus::M100FlightStatus.
 */
typedef uint8_t  M100Status;  // pack(1)
/*!
 * @brief Return type for battery data broadcast (only for M100). Returns percentage.
 */
typedef uint8_t  M100Battery; // pack(1)

#pragma pack()

extern TopicInfo TopicDataBase[];

/*! @brief template struct maps a topic name to the corresponding data
 * type
 *
 */
template <TopicName T>
struct TypeMap
{
  typedef void type;
};

// clang-format off
template <> struct TypeMap<TOPIC_QUATERNION               > { typedef Quaternion      type;};
template <> struct TypeMap<TOPIC_ACCELERATION_GROUND      > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ACCELERATION_BODY        > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ACCELERATION_RAW         > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_VELOCITY                 > { typedef Velocity        type;};
template <> struct TypeMap<TOPIC_ANGULAR_RATE_FUSIONED    > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ANGULAR_RATE_RAW         > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_ALTITUDE_FUSIONED        > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_ALTITUDE_BAROMETER       > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_HEIGHT_HOMEPOINT         > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_HEIGHT_FUSION            > { typedef float32_t       type;};
template <> struct TypeMap<TOPIC_GPS_FUSED                > { typedef GPSFused        type;};
template <> struct TypeMap<TOPIC_GPS_DATE                 > { typedef uint32_t        type;};
template <> struct TypeMap<TOPIC_GPS_TIME                 > { typedef uint32_t        type;};
template <> struct TypeMap<TOPIC_GPS_POSITION             > { typedef Vector3d        type;};
template <> struct TypeMap<TOPIC_GPS_VELOCITY             > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_GPS_DETAILS              > { typedef GPSDetail       type;};
template <> struct TypeMap<TOPIC_RTK_POSITION             > { typedef PositionData    type;};
template <> struct TypeMap<TOPIC_RTK_VELOCITY             > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_RTK_YAW                  > { typedef int16_t         type;};
template <> struct TypeMap<TOPIC_RTK_POSITION_INFO        > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_RTK_YAW_INFO             > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_COMPASS                  > { typedef Mag             type;};
template <> struct TypeMap<TOPIC_RC                       > { typedef RC              type;};
template <> struct TypeMap<TOPIC_GIMBAL_ANGLES            > { typedef Vector3f        type;};
template <> struct TypeMap<TOPIC_GIMBAL_STATUS            > { typedef GimbalStatus    type;};
template <> struct TypeMap<TOPIC_STATUS_FLIGHT            > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_STATUS_DISPLAYMODE       > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_STATUS_LANDINGGEAR       > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_STATUS_MOTOR_START_ERROR > { typedef uint16_t        type;};
template <> struct TypeMap<TOPIC_BATTERY_INFO             > { typedef Battery         type;};
template <> struct TypeMap<TOPIC_CONTROL_DEVICE           > { typedef SDKInfo         type;};
template <> struct TypeMap<TOPIC_HARD_SYNC                > { typedef HardSyncData    type;};
template <> struct TypeMap<TOPIC_GPS_SIGNAL_LEVEL         > { typedef uint8_t         type;};
template <> struct TypeMap<TOPIC_GPS_CONTROL_LEVEL        > { typedef uint8_t         type;};
// clang-format on
}
}
}
#endif // ONBOARDSDK_DJI_Telemetry_H
