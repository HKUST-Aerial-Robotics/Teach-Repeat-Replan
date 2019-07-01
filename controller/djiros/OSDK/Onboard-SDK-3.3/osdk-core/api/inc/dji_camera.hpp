/** @file dji_camera.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Camera/Gimbal API for DJI onboardSDK library
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#ifndef DJI_CAMERA_H
#define DJI_CAMERA_H

#include "dji_command.hpp"
#include "dji_type.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declaration
class Vehicle;

/*! @brief Camera class for controlling camera-related functions
 * available through open protocol
 *
 */
class Camera
{
public:
  Camera(Vehicle* vehicle);
  ~Camera();

public:
  // Non-Blocking API
  /*! take a photo, check DJI Go app or SD card for photo */
  void shootPhoto();
  /*! start recording video, check DJI Go app or SD card for video */
  void videoStart();
  /*! stop recording video, check DJI Go app or SD card for video */
  void videoStop();

private:
  /*! @brief Function for commanding: Take Picture, Start Video, Stop Video
   *  @note The camera function does not return an acknowledgment.
   *  @param cmd array representing camera command
   *  Available camera commands:
   *  OpenProtocol::CMDSet::Control::cameraShot
   *  OpenProtocol::CMDSet::Control::cameraVideoStart
   *  OpenProtocol::CMDSet::Control::cameraVideoStop
   */
  void action(const uint8_t cmd[]);

private:
  Vehicle* vehicle;
}; // class camera
} // namespace OSDK
} // namespace DJI

#endif // DJI_CAMERA_H
