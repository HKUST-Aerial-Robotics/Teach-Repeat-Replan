/** @file dji_sdk_node_services.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the general services of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

bool
DJISDKNode::droneActivationCallback(dji_sdk::Activation::Request&  request,
                                    dji_sdk::Activation::Response& response)
{
  ROS_DEBUG("called droneActivationCallback");

  //! @note activation arguments should be specified in launch files
  ACK::ErrorCode ack;
  ack = this->activate(this->app_id, this->enc_key);

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response.result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response.result = true;
    ROS_DEBUG("drone activated");
  }

  return true;
}

bool
DJISDKNode::droneArmCallback(dji_sdk::DroneArmControl::Request&  request,
                             dji_sdk::DroneArmControl::Response& response)
{
  ROS_DEBUG("called droneArmCallback");

  ACK::ErrorCode ack;

  if (request.arm)
  {
    ack = vehicle->control->armMotors(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->armMotors()");
  }
  else
  {
    ack = vehicle->control->disArmMotors(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->disArmMotors()");
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response.result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
DJISDKNode::sdkCtrlAuthorityCallback(
  dji_sdk::SDKControlAuthority::Request&  request,
  dji_sdk::SDKControlAuthority::Response& response)
{

  ROS_DEBUG("called sdkCtrlAuthorityCallback");

  ACK::ErrorCode ack;
  if (request.control_enable)
  {
    ack = vehicle->obtainCtrlAuthority(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->obtainCtrlAuthority");
  }
  else
  {
    ack = vehicle->releaseCtrlAuthority(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->releaseCtrlAuthority");
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    response.result = false;
    ACK::getErrorCodeMessage(ack, __func__);
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
DJISDKNode::droneTaskCallback(dji_sdk::DroneTaskControl::Request&  request,
                              dji_sdk::DroneTaskControl::Response& response)
{

  ROS_DEBUG("called droneTaskCallback");

  ACK::ErrorCode ack;
  if (request.task == 4)
  {
    // takeoff
    ack = vehicle->control->takeoff(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->takeoff()");
  }
  else if (request.task == 6)
  {
    // landing
    ack = vehicle->control->land(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->land()");
  }
  else if (request.task == 1)
  {
    // gohome
    ack = vehicle->control->goHome(WAIT_TIMEOUT);
    ROS_DEBUG("called vehicle->control->goHome()");
  }
  else
  {
    ROS_WARN("unknown request task in droneTaskCallback");
    response.result = false;
  }

  ROS_DEBUG("ack.info: set=%i id=%i", ack.info.cmd_set, ack.info.cmd_id);
  ROS_DEBUG("ack.data: %i", ack.data);

  response.cmd_set  = (int)ack.info.cmd_set;
  response.cmd_id   = (int)ack.info.cmd_id;
  response.ack_data = (unsigned int)ack.data;

  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    response.result = false;
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
DJISDKNode::cameraActionCallback(dji_sdk::CameraAction::Request&  request,
                                 dji_sdk::CameraAction::Response& response)
{
  ROS_DEBUG("called cameraActionCallback");

  if (request.camera_action == 0)
  {
    vehicle->camera->shootPhoto();
    response.result = true;
  }
  else if (request.camera_action == 1)
  {
    vehicle->camera->videoStart();
    response.result = true;
  }
  else if (request.camera_action == 2)
  {
    vehicle->camera->videoStop();
    response.result = true;
  }
  else
  {
    ROS_WARN("unknown request task in cameraActionCallback");
    response.result = false;
  }

  return true;
}

bool
DJISDKNode::MFIOConfigCallback(dji_sdk::MFIOConfig::Request&  request,
                               dji_sdk::MFIOConfig::Response& response)
{
  ROS_DEBUG("called MFIOConfigCallback");

  vehicle->mfio->config((MFIO::MODE)request.mode,
                        (MFIO::CHANNEL)request.channel,
                        (uint32_t)request.init_on_time_us,
                        (uint16_t)request.pwm_freq, WAIT_TIMEOUT);
}

bool
DJISDKNode::MFIOSetValueCallback(dji_sdk::MFIOSetValue::Request&  request,
                                 dji_sdk::MFIOSetValue::Response& response)
{
  ROS_DEBUG("called MFIOSetValueCallback");

  vehicle->mfio->setValue((MFIO::CHANNEL)request.channel,
                          (uint32_t)request.init_on_time_us, WAIT_TIMEOUT);
}

bool
DJISDKNode::setHardsyncCallback(dji_sdk::SetHardSync::Request&  request,
                                dji_sdk::SetHardSync::Response& response)
{
  ROS_DEBUG("called setHardsyncCallback");
  if (request.frequency == 0)
  {
    ROS_INFO("Call setSyncFreq with parameters (freq=%d, tag=%d). Will do one "
             "time trigger...",
             request.frequency, request.tag);
    vehicle->hardSync->setSyncFreq(request.frequency, request.tag);
    response.result = true;
    return true;
  }

  // The frequency must be between 0 and 200, and be a divisor of 400
  if (request.frequency > 0 && request.frequency <= 200)
  {
    if (400 % (request.frequency) == 0)
    {
      ROS_INFO("Call setSyncFreq with parameters (freq=%d, tag=%d).",
               request.frequency, request.tag);
      vehicle->hardSync->setSyncFreq(request.frequency, request.tag);
      response.result = true;
      return true;
    }
  }

  ROS_INFO("In valid frequency!");
  response.result = false;
  return true;
}
