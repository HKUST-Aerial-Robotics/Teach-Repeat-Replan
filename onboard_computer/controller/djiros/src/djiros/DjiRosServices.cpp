//
// Created by ltb on 6/8/17.
//

#include <djiros/DjiRos.h>

bool DjiRos::initServices(ros::NodeHandle &nh) {
  drone_arm_server = nh.advertiseService("drone_arm_control", &DjiRos::droneArmCallback, this);
  sdk_ctrlAuthority_server  = nh.advertiseService("sdk_control_authority", &DjiRos::sdkCtrlAuthorityCallback, this);
  // ctrl_sub = nh.subscribe<sensor_msgs::Joy>("ctrl",
  //                                           10,
  //                                           boost::bind(&DjiRos::control_callback, this, _1),
  //                                           ros::VoidConstPtr(),
  //                                           ros::TransportHints().tcpNoDelay());


//    gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
//            "dji_sdk/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
//    gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
//            "dji_sdk/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);

  return true;
};

bool
DjiRos::droneArmCallback(dji_sdk::DroneArmControl::Request&  request,
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
DjiRos::sdkCtrlAuthorityCallback(
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
