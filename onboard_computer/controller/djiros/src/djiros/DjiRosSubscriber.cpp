//
// Created by ltb on 6/8/17.
//

#include <djiros/DjiRos.h>

bool DjiRos::initSubscriber(ros::NodeHandle &nh) {
  ctrl_sub = nh.subscribe<sensor_msgs::Joy>("ctrl",
                                            10,
                                            boost::bind(&DjiRos::control_callback, this, _1),
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().tcpNoDelay());


//    gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
//            "dji_sdk/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
//    gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
//            "dji_sdk/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);

  return true;
};

void DjiRos::control_callback(const sensor_msgs::JoyConstPtr &pMsg) {
    if (pMsg->header.frame_id.compare("FRD") == 0) {
    last_ctrl_stamp = ros::Time::now();

    uint8_t flag = 0;
    if (pMsg->axes[4] > 0) {
      flag = 0b00100010;  // 0b00100000, mode 13
      if (pMsg->axes.size() > 5 && pMsg->axes.at(5) > 0.5) {
        flag = 0b00101010; // 0b00100000, mode 14 yaw_rate  
      }
      else {
        flag = 0b00100010; // 0b00100000, yaw_angel 
      }
    } else {
      flag = 0b00000010;  // 0b00000000, mode 1
    }

  // enum YawLogic
  // {
  //    /*!
  //    - Set the control-mode to control yaw angle.
  //    - Yaw angle is referenced to the ground frame.
  //    - In this control mode, Ground frame is enforeced in Autopilot.
  //      */
  //   YAW_ANGLE = 0x00,
  //   /*!
  //    - Set the control-mode to control yaw angular velocity.
  //    - Same reference frame as YAW_ANGLE.
  //    - Limite: 150 deg/s
  //    */
  //   YAW_RATE = 0x08
  // };

    // ROS_WARN("YAW_ANGLE = %d",DJI::OSDK::Control::YAW_ANGLE);  // YAW_ANGLE == 0 //zxzxzxzx
    // ROS_WARN("YAW_RATE = %d",DJI::OSDK::Control::YAW_RATE);
    // ROS_WARN("flag = %X",flag);
    // if((flag & 0x08) == DJI::OSDK::Control::YAW_ANGLE) ROS_WARN("in YAW_ANGLE");
    // else ROS_WARN("in YAW_RATE");
    DJI::OSDK::Control::CtrlData ctrl_data(flag, pMsg->axes[0], pMsg->axes[1], pMsg->axes[2], pMsg->axes[3]);

    vehicle->control->flightCtrl(ctrl_data);

    // if (pMsg->buttons.size()) {
    //     ros::Time feedback_stamp;
    //     feedback_stamp.sec = pMsg->buttons[1] * pMsg->buttons[0] + pMsg->buttons[2];
    //     feedback_stamp.nsec = pMsg->buttons[3] * pMsg->buttons[0] + pMsg->buttons[4];
    //     ROS_INFO("curr: %.3f fbk: %.3f dt=%.3f", last_ctrl_time.toSec(),
    //     feedback_stamp.toSec(),
    //              (last_ctrl_time - feedback_stamp).toSec());
    // }
    } else {
    ROS_ERROR("[djiros] input joy_msg.frame_id should be FRD!!!");
  }
}