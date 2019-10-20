/** @file dji_gimbal.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Gimbal API for OSDK library
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef GIMBAL_H
#define GIMBAL_H

#include "dji_command.hpp"
#include "dji_type.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declaration
class Vehicle;

/*!
 * @brief The Gimbal class for controlling gimbal-related functions
 */
class Gimbal
{
public:
#pragma pack(1)
  /*! @brief The Angle Data struct for gimbal control.

      The relationship bewteen the angle reference in absolute control mode and
     gimbal mode configuration in DJI Go App
        Gimbal Mode |   Roll    |   Pitch   |   Yaw |   Gimbal Follow UAV's Head
        ----------  |---------  |---------  |-----  |------------------
        Follow      | 	Ground  | 	Ground  |Body   | 	Y
        FPV         |	N/A     | 	Ground  | N/A   | 	Y
        Free        | 	Ground  | 	Ground  | Ground| 	N
      @note
            - Rotating 90 degree in pitch direction will cause gimbal lock
     problem, in which the value of roll and yaw are not reliable.
            - M600 with A3 only supports Ronin-MX/Zenmuse X5 Series/Zenmuse
     X3/Zenmuse XT when operating gimbal control in angle.

   */
  typedef struct AngleData
  {
    int16_t yaw;   /*!< Yaw angle, unit 0.1 degree , input range [-3200,3200] */
    int16_t roll;  /*!< Roll angle, unit 0.1 degree, input range [-350,350] */
    int16_t pitch; /*!< Pitch angle, unit 0.1 degree, input range [-900,300] */
    uint8_t mode;
    // clang-format off
    /*!< Mode is 1 byte size:
        Bit #:        | Set to 0: | Set to 1:
        ------------- | ------------- | -------------
        bit 0         |  Incremental control, the angle reference is the current Gimbal location    |  Absolute control, the angle reference is related to configuration in DJI Go App
        bit 1         | Gimbal will follow the command in Yaw |  Gimbal will maintain position in Yaw
        bit 2         | Roll invalid bit, the same as bit[1] | Roll invalid bit, the same as bit[1]|
        bit 3         |Pitch invalid bit, the same as bit[1] | Pitch invalid bit, the same as bit[1]
        bit [4:7]     | bit [4:7]: reserved, set to be 0| |
    */
    // clang-format on
    uint8_t duration; /*!<  Command completion time.
                            - Unit 0.1s, for example 20 means gimbal will reach
                         the commended postition in 2 seconds.
                            - Rotate rate beyond 400º/s is not recommand */
  } AngleData;        // pack(1)

  /*! @brief The Speed Data struct for gimbal control
   *
   *  The angle reference frame is same in AngleData free mode.
   */
  typedef struct SpeedData
  {
    int16_t yaw;  /*!< Yaw in rate, unit 0.1 deg/s, input range[-1800,1800] */
    int16_t roll; /*!< Roll in rate, unit 0.1 deg/s, input range[-1800,1800] */
    int16_t
            pitch; /*!< Pitch in rate, unit 0.1 deg/s, input range[-1800,1800] */
    uint8_t reserved; /*!< @note always 0x80 */
  } SpeedData;        // pack(1)
#pragma pack()
public:
  Gimbal(Vehicle* vehicle);
  ~Gimbal();

public:
  /*! @brief Function for setting Gimbal Angle
   *
   *  Define the mode and the angles for the gimbal angle control in the
   * AngleData struct and pass it to setAngle().
   *
   *  @param data AngleData struct containing all the angle values, mode
   *  and duration
   *  @return void
   */
  void setAngle(Gimbal::AngleData* data);

  /*! @brief Function for setting Gimbal Speed
   *
   *  Define the rate of change for the gimbal angle in SpeedData struct and
   * pass it to setSpeed();
   *
   *  @param data SpeedData struct containing the roll, pitch and yaw
   *  rates
   *  @return void
   */
  void setSpeed(Gimbal::SpeedData* data);

private:
  Vehicle* vehicle;
};

} // OSDK
} // DJI

#endif // GIMBAL_H
