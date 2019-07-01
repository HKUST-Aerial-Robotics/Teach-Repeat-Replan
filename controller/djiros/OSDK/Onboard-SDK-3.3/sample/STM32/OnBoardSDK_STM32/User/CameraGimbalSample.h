#ifndef CAMERAGIMBALSAMPLE_H
#define CAMERAGIMBALSAMPLE_H

#include "timer.h"
#include <cstdint>
#include <dji_camera.hpp>
#include <dji_gimbal.hpp>
#include <dji_vehicle.hpp>
#include <stdio.h>

struct RotationAngle
{
  DJI::OSDK::float32_t roll;
  DJI::OSDK::float32_t pitch;
  DJI::OSDK::float32_t yaw;

  RotationAngle(DJI::OSDK::float32_t roll = 0, DJI::OSDK::float32_t pitch = 0,
                DJI::OSDK::float32_t yaw = 0)
    : roll(roll)
    , pitch(pitch)
    , yaw(yaw)
  {
  }
};

struct GimbalContainer
{
  DJI::OSDK::float32_t roll;
  DJI::OSDK::float32_t pitch;
  DJI::OSDK::float32_t yaw;
  int                  duration;
  int                  isAbsolute;
  bool                 yaw_cmd_ignore;
  bool                 pitch_cmd_ignore;
  bool                 roll_cmd_ignore;
  RotationAngle        initialAngle;
  RotationAngle        currentAngle;
  GimbalContainer(int roll = 0, int pitch = 0, int yaw = 0, int duration = 0,
                  int isAbsolute = 0, bool yaw_cmd_ignore = false,
                  bool pitch_cmd_ignore = false, bool roll_cmd_ignore = false,
                  RotationAngle initialAngle = RotationAngle(),
                  RotationAngle currentAngle = RotationAngle())
    : roll(roll)
    , pitch(pitch)
    , yaw(yaw)
    , duration(duration)
    , isAbsolute(isAbsolute)
    , yaw_cmd_ignore(yaw_cmd_ignore)
    , pitch_cmd_ignore(pitch_cmd_ignore)
    , roll_cmd_ignore(roll_cmd_ignore)
    , initialAngle(initialAngle)
    , currentAngle(currentAngle)
  {
  }
};

// Helper functions
void doSetGimbalAngle(GimbalContainer* gimbal);
bool gimbalCameraControl();
void displayResult(RotationAngle* currentAngle);

#endif // CAMERAGIMBALSAMPLE_H
