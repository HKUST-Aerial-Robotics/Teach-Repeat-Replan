/** @file dji_sdk_node_subscriber.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the subscribers of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

void
DJISDKNode::gimbalAngleCtrlCallback(const dji_sdk::Gimbal::ConstPtr& msg)
{
  ROS_DEBUG("called gimbalAngleCtrlCallback");

  DJI::OSDK::Gimbal::AngleData angle_data;
  //! OSDK takes 0.1 sec as unit
  angle_data.duration = msg->ts*10;
  angle_data.mode     = msg->mode;
  //! OSDK takes 0.1 deg as unit
  angle_data.roll     = RAD2DEG(msg->roll)*10;
  angle_data.pitch    = RAD2DEG(msg->pitch)*10;
  angle_data.yaw      = RAD2DEG(msg->yaw)*10;
  vehicle->gimbal->setAngle(&angle_data);
}

void
DJISDKNode::gimbalSpeedCtrlCallback(
  const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
  ROS_DEBUG("called gimbalAngleCtrlCallback");

  DJI::OSDK::Gimbal::SpeedData speed_data;
  //! OSDK takes 0.1 deg as unit
  speed_data.roll  = RAD2DEG(msg->vector.y)*10;
  speed_data.pitch = RAD2DEG(msg->vector.x)*10;
  speed_data.yaw   = RAD2DEG(msg->vector.z)*10;
  vehicle->gimbal->setSpeed(&speed_data);
}
