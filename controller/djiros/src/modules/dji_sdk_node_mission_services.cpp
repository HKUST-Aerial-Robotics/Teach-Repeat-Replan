/** @file dji_sdk_node_mission_services.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the mission functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

bool
DJISDKNode::missionStatusCallback(dji_sdk::MissionStatus::Request&  request,
                                  dji_sdk::MissionStatus::Response& response)
{
  ROS_DEBUG("called missionStatusCallback");

  response.waypoint_mission_count = vehicle->missionManager->wayptCounter;
  response.hotpoint_mission_count = vehicle->missionManager->hotptCounter;
}

bool
DJISDKNode::missionWpUploadCallback(
  dji_sdk::MissionWpUpload::Request&  request,
  dji_sdk::MissionWpUpload::Response& response)
{
  ROS_DEBUG("called missionWpUpload");

  //! initialize waypoint mission related info
  ACK::ErrorCode                  initAck;
  DJI::OSDK::WayPointInitSettings wpInitData;
  wpInitData.indexNumber  = (unsigned char)request.waypoint_task.mission_waypoint.size();
  wpInitData.maxVelocity  = (float)request.waypoint_task.velocity_range;
  wpInitData.idleVelocity = (float)request.waypoint_task.idle_velocity;
  wpInitData.finishAction = (unsigned char)request.waypoint_task.action_on_finish;
  wpInitData.executiveTimes = (unsigned char)request.waypoint_task.mission_exec_times;
  wpInitData.yawMode        = (unsigned char)request.waypoint_task.yaw_mode;
  wpInitData.traceMode      = (unsigned char)request.waypoint_task.trace_mode;
  wpInitData.RCLostAction   = (unsigned char)request.waypoint_task.action_on_rc_lost;
  wpInitData.gimbalPitch    = (unsigned char)request.waypoint_task.gimbal_pitch_mode;
  wpInitData.latitude = 0.0;
  wpInitData.longitude = 0.0;
  wpInitData.altitude = 0.0;
  for (int i = 0; i < 16; i++){  
    wpInitData.reserved[i] = 0;
  } 

  initAck = vehicle->missionManager->init(DJI_MISSION_TYPE::WAYPOINT,
                                          WAIT_TIMEOUT, &wpInitData);

  ROS_DEBUG("ack.info: set=%i id=%i", initAck.info.cmd_set,
            initAck.info.cmd_id);
  ROS_DEBUG("ack.data: %i", initAck.data);

  response.cmd_set  = (int)initAck.info.cmd_set;
  response.cmd_id   = (int)initAck.info.cmd_id;
  response.ack_data = (unsigned int)initAck.data;

  if (ACK::getError(initAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
    response.result = false;
  }

  ROS_INFO("initialized waypoint mission");
  sleep(1);

  //! initialize waypoint mission related info
  ACK::WayPointIndex          uploadAck;
  DJI::OSDK::WayPointSettings wpData;
  int                         i = 0;
  for (auto waypoint : request.waypoint_task.mission_waypoint)
  {
    wpData.latitude        = waypoint.latitude * C_PI / 180;
    wpData.longitude       = waypoint.longitude * C_PI / 180;
    wpData.altitude        = waypoint.altitude;
    wpData.damping         = waypoint.damping_distance;
    wpData.yaw             = waypoint.target_yaw;
    wpData.gimbalPitch     = waypoint.target_gimbal_pitch;
    wpData.turnMode        = waypoint.turn_mode;
    wpData.hasAction       = waypoint.has_action;
    wpData.actionTimeLimit = waypoint.action_time_limit;
    wpData.actionNumber    = 15;
    wpData.actionRepeat    = waypoint.waypoint_action.action_repeat;
    wpData.index           = i;
    std::copy(waypoint.waypoint_action.command_list.begin(),
              waypoint.waypoint_action.command_list.end(), wpData.commandList);
    std::copy(waypoint.waypoint_action.command_parameter.begin(),
              waypoint.waypoint_action.command_parameter.end(),
              wpData.commandParameter);

    uploadAck = vehicle->missionManager->wpMission->uploadIndexData(
      &wpData, WAIT_TIMEOUT);

    ROS_DEBUG("uploaded waypoint lat: %f lon: %f alt: %f", waypoint.latitude,
              waypoint.longitude, waypoint.altitude);

    response.cmd_set  = (int)uploadAck.ack.info.cmd_set;
    response.cmd_id   = (int)uploadAck.ack.info.cmd_id;
    response.ack_data = (unsigned int)uploadAck.ack.data;

    if (ACK::getError(uploadAck.ack))
    {
      ACK::getErrorCodeMessage(uploadAck.ack, __func__);
      response.result = false;
    }
    else
    {
      response.result = true;
    }

    ROS_INFO("uploaded the %dth waypoint\n", (wpData.index + 1));
    i += 1;
    sleep(1);
  }

  ROS_INFO("waypoint mission initialized and uploaded");
  return true;
}

bool
DJISDKNode::missionWpActionCallback(
  dji_sdk::MissionWpAction::Request&  request,
  dji_sdk::MissionWpAction::Response& response)
{
  ROS_DEBUG("called missionWpActionCallback");

  if (vehicle->missionManager->wayptCounter == 0)
  {
    ROS_ERROR("no waypoint mission uploaded");
    response.result = false;
  }

  ACK::ErrorCode ack;
  switch (request.action)
  {
    case DJI::OSDK::MISSION_ACTION::START:
      ack = vehicle->missionManager->wpMission->start(WAIT_TIMEOUT);
      ROS_DEBUG("start waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::STOP:
      ack = vehicle->missionManager->wpMission->stop(WAIT_TIMEOUT);
      ROS_DEBUG("stop waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::PAUSE:
      ack = vehicle->missionManager->wpMission->pause(WAIT_TIMEOUT);
      ROS_DEBUG("pause waypoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::RESUME:
      ack = vehicle->missionManager->wpMission->resume(WAIT_TIMEOUT);
      ROS_DEBUG("resume waypoint mission");
      break;
    default:
      ROS_WARN("unknown action specified in MissionWpAction service");
      break;
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
DJISDKNode::missionWpGetSpeedCallback(
  dji_sdk::MissionWpGetSpeed::Request&  request,
  dji_sdk::MissionWpGetSpeed::Response& response)
{
  ROS_DEBUG("called wpGetSpeedCallback");

  if (vehicle->missionManager->wayptCounter > 0)
  {
    //! @todo bug here
    //    response.speed =
    //      (vehicle->missionManager->wpMission->readIdleVelocity(WAIT_TIMEOUT))
    //        .idleVelocity;
    //    vehicle->missionManager->wpMission->readIdleVelocity();
  }
  else
  {
    ROS_ERROR("no waypoint mission initiated ");
  }
  // @todo some bug in FC side, need to follow up
  std::cout << "response.speed " << response.speed << std::endl;

  return true;
}

bool
DJISDKNode::missionWpSetSpeedCallback(
  dji_sdk::MissionWpSetSpeed::Request&  request,
  dji_sdk::MissionWpSetSpeed::Response& response)
{
  ROS_DEBUG("called wpSetSpeedCallback");

  ACK::WayPointVelocity velAck;

  if (vehicle->missionManager->wayptCounter > 0)
  {
    velAck = (vehicle->missionManager->wpMission->updateIdleVelocity(
      request.speed, WAIT_TIMEOUT));
  }
  else
  {
    ROS_ERROR("no waypoint mission initiated ");
    response.result = false;
  }

  if (ACK::getError(velAck.ack))
  {
    ROS_DEBUG("wpSetSpeedCallback ack value: %d", (uint32_t)velAck.ack.data);
    response.result = false;
  }
  else
  {
    response.result = true;
  }

  return true;
}

bool
DJISDKNode::missionWpGetInfoCallback(
  dji_sdk::MissionWpGetInfo::Request&  request,
  dji_sdk::MissionWpGetInfo::Response& response)
{
  ROS_DEBUG("called missionWpGetInfoCallback");

  DJI::OSDK::WayPointInitSettings info;
  if (vehicle->missionManager->wayptCounter > 0)
  {
   // info = vehicle->missionManager->wpMission->getInfo();
  }
  else
  {
    ROS_ERROR("no waypoint mission initiated ");
  }

  response.waypoint_task.mission_waypoint.resize(info.indexNumber);
  response.waypoint_task.velocity_range     = info.maxVelocity;
  response.waypoint_task.idle_velocity      = info.idleVelocity;
  response.waypoint_task.action_on_finish   = info.finishAction;
  response.waypoint_task.mission_exec_times = info.executiveTimes;
  response.waypoint_task.yaw_mode           = info.yawMode;
  response.waypoint_task.trace_mode         = info.traceMode;
  response.waypoint_task.action_on_rc_lost  = info.RCLostAction;
  response.waypoint_task.gimbal_pitch_mode  = info.gimbalPitch;

  return true;
}

bool
DJISDKNode::missionHpUploadCallback(
  dji_sdk::MissionHpUpload::Request&  request,
  dji_sdk::MissionHpUpload::Response& response)
{
  ROS_DEBUG("called missionHpUploadCallback");

  DJI::OSDK::HotPointSettings* hpInitData = new DJI::OSDK::HotPointSettings();
  hpInitData->latitude   = request.hotpoint_task.latitude * C_PI / 180;
  hpInitData->longitude  = request.hotpoint_task.longitude * C_PI / 180;
  hpInitData->height     = request.hotpoint_task.altitude;
  hpInitData->radius     = request.hotpoint_task.radius;
  hpInitData->yawRate    = request.hotpoint_task.angular_speed;
  hpInitData->clockwise  = request.hotpoint_task.is_clockwise;
  hpInitData->startPoint = request.hotpoint_task.start_point;
  hpInitData->yawMode    = request.hotpoint_task.yaw_mode;

  vehicle->missionManager->init(DJI_MISSION_TYPE::HOTPOINT, WAIT_TIMEOUT,
                                (void*)hpInitData);

  response.result = true;
  return true;
}

bool
DJISDKNode::missionHpActionCallback(
  dji_sdk::MissionHpAction::Request&  request,
  dji_sdk::MissionHpAction::Response& response)
{
  ROS_DEBUG("called missionHpActionCallback");

  if (vehicle->missionManager->hotptCounter == 0)
  {
    ROS_ERROR("no hotpoint mission uploaded");
    response.result = false;
  }

  ACK::ErrorCode ack;
  switch (request.action)
  {
    case DJI::OSDK::MISSION_ACTION::START:
      ack = vehicle->missionManager->hpMission->start(WAIT_TIMEOUT);
      ROS_DEBUG("start hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::STOP:
      ack = vehicle->missionManager->hpMission->stop(WAIT_TIMEOUT);
      ROS_DEBUG("stop hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::PAUSE:
      ack = vehicle->missionManager->hpMission->pause(WAIT_TIMEOUT);
      ROS_DEBUG("pause hotpoint mission");
      break;
    case DJI::OSDK::MISSION_ACTION::RESUME:
      ack = vehicle->missionManager->hpMission->resume(WAIT_TIMEOUT);
      ROS_DEBUG("resume hotpoint mission");
      break;
    default:
      ROS_WARN("unknown action specified in MissionHpAction service");
      break;
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
DJISDKNode::missionHpGetInfoCallback(
  dji_sdk::MissionHpGetInfo::Request&  request,
  dji_sdk::MissionHpGetInfo::Response& response)
{
  ROS_DEBUG("called missionHpGetInfoCallback");

  DJI::OSDK::HotPointSettings info;
  if (vehicle->missionManager->hotptCounter > 0)
  {
    info = vehicle->missionManager->hpMission->getData();
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
  }

  response.hotpoint_task.latitude      = info.latitude;
  response.hotpoint_task.longitude     = info.longitude;
  response.hotpoint_task.altitude      = info.height;
  response.hotpoint_task.radius        = info.radius;
  response.hotpoint_task.angular_speed = info.yawRate;
  response.hotpoint_task.is_clockwise  = info.clockwise;
  response.hotpoint_task.start_point   = info.startPoint;
  response.hotpoint_task.yaw_mode      = info.yawMode;

  return true;
}

bool
DJISDKNode::missionHpUpdateYawRateCallback(
  dji_sdk::MissionHpUpdateYawRate::Request&  request,
  dji_sdk::MissionHpUpdateYawRate::Response& response)
{
  ROS_DEBUG("called missionHpUpdateYawRateCallback");

  DJI::OSDK::HotpointMission::YawRate yawRate;
  yawRate.yawRate   = request.yaw_rate;
  yawRate.clockwise = request.direction;

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hotptCounter > 0)
  {
    ack =
      vehicle->missionManager->hpMission->updateYawRate(yawRate, WAIT_TIMEOUT);
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
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
DJISDKNode::missionHpResetYawCallback(
  dji_sdk::MissionHpResetYaw::Request&  request,
  dji_sdk::MissionHpResetYaw::Response& response)
{
  ROS_DEBUG("called missionHpResetYawCallback");

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hotptCounter > 0)
  {
    ack = vehicle->missionManager->hpMission->resetYaw(WAIT_TIMEOUT);
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
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
DJISDKNode::missionHpUpdateRadiusCallback(
  dji_sdk::MissionHpUpdateRadius::Request&  request,
  dji_sdk::MissionHpUpdateRadius::Response& response)
{
  ROS_DEBUG("called missionHpUpdateRadiusCallback");

  ACK::ErrorCode ack;
  if (vehicle->missionManager->hotptCounter > 0)
  {
    ack = vehicle->missionManager->hpMission->updateRadius(request.radius,
                                                           WAIT_TIMEOUT);
  }
  else
  {
    ROS_ERROR("no hotpoint mission initiated ");
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
