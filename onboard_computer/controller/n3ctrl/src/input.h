#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>
#include <stdint.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <uav_utils/utils.h>
#include "N3CtrlParam.h"

class RC_Data_t {
  public:
    double roll;
    double pitch;
    double yaw;
    double thr;
    double mode;
    double gear;

    double last_mode;
    double last_gear;

    sensor_msgs::Joy msg;
    ros::Time rcv_stamp;

    bool is_command_mode;
    bool enter_command_mode;
    bool is_api_mode;
    bool enter_api_mode;

    static constexpr double MANUAL_MODE_GEAR_VALUE = -1.0;      // gear up on M100RC
    static constexpr double COMMAND_MODE_GEAR_VALUE = -0.4545;  // gear down on M100RC
    static constexpr double GEAR_SHIFT_VALUE = -0.6;

    // api mode on M100RC, normal value is 0.8
    static constexpr double API_MODE_THRESHOLD_VALUE = 0.5;

    RC_Data_t();
    void check_validity();
    void set_default_mode(std::string s);
    void feed(sensor_msgs::JoyConstPtr pMsg);
    bool check_enter_command_mode();
};

class Odom_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d w;
    Eigen::Quaterniond q;

    nav_msgs::Odometry msg;
    ros::Time rcv_stamp;

    Odom_Data_t();
    void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Imu_Data_t {
  public:
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
    Eigen::Vector3d a;

    sensor_msgs::Imu msg;
    ros::Time rcv_stamp;

    Imu_Data_t();
    void feed(sensor_msgs::ImuConstPtr pMsg);
};

class Command_Data_t {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d a;
    double yaw;
    uint32_t trajectory_id;
    uint8_t trajectory_flag;

    quadrotor_msgs::PositionCommand msg;
    ros::Time rcv_stamp;

    Command_Data_t();
    void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Idling_Data_t {
  public:
    bool need_idling;

    geometry_msgs::Vector3Stamped msg;
    ros::Time rcv_stamp;

    Idling_Data_t();
    void feed(geometry_msgs::Vector3StampedConstPtr pMsg);
};

class Trigger_Data_t {
  public:
    bool need_enter_js;

    Trigger_Data_t();
    void feed(std_msgs::HeaderConstPtr pMsg);
    bool get_enter_js();
};

#endif