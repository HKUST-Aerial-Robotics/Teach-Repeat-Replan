/*! @file camera_gimbal_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Camera and Gimbal Control API usage in a Linux environment.
 *  Shows example usage of camera commands and gimbal position/speed control
 * APIs
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef DJIOSDK_CAMERAGIMBALSAMPLE_HPP
#define DJIOSDK_CAMERAGIMBALSAMPLE_HPP

// DJI OSDK includes
#include <dji_linux_helpers.hpp>
#include <dji_vehicle.hpp>

// Be precise here
struct RotationAngle
{
  DJI::OSDK::float32_t roll;
  DJI::OSDK::float32_t pitch;
  DJI::OSDK::float32_t yaw;
};

struct GimbalContainer
{
  int           roll             = 0;
  int           pitch            = 0;
  int           yaw              = 0;
  int           duration         = 0;
  int           isAbsolute       = 0;
  bool          yaw_cmd_ignore   = false;
  bool          pitch_cmd_ignore = false;
  bool          roll_cmd_ignore  = false;
  RotationAngle initialAngle;
  RotationAngle currentAngle;
  GimbalContainer(int roll = 0, int pitch = 0, int yaw = 0, int duration = 0,
                  int isAbsolute = 0, RotationAngle initialAngle = {},
                  RotationAngle currentAngle = {})
    : roll(roll)
    , pitch(pitch)
    , yaw(yaw)
    , duration(duration)
    , isAbsolute(isAbsolute)
    , initialAngle(initialAngle)
    , currentAngle(currentAngle)
  {
  }
};

// Helper functions
void doSetGimbalAngle(DJI::OSDK::Vehicle* vehicle, GimbalContainer* gimbal);
bool gimbalCameraControl(DJI::OSDK::Vehicle* vehicle);
void displayResult(RotationAngle* currentAngle);

#endif // DJIOSDK_CAMERAGIMBALSAMPLE_HPP
