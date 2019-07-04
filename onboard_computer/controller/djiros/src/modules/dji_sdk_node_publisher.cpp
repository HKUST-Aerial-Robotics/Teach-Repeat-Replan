/** @file dji_sdk_node_publisher.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the publishers of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>
#include <tf/tf.h>

#include <sensor_msgs/Joy.h>

#define _TICK2ROSTIME(tick) (ros::Duration((double)(tick) / 1000.0))


void
DJISDKNode::SDKBroadcastCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                 DJI::OSDK::UserData userData)
{
  ((DJISDKNode*)userData)->dataBroadcastCallback();
}

void
DJISDKNode::dataBroadcastCallback()
{
  using namespace DJI::OSDK;

  ros::Time now_time = ros::Time::now();

  short int data_enable_flag = vehicle->broadcast->getPassFlag();

  /*!
   * rc gear reading will be positive when no RC is connected
   */
  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_RC)
  {
    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = now_time;
    rc_joy.header.frame_id = "rc";

    rc_joy.axes.reserve(6);
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().roll     / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().pitch    / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().yaw      / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().throttle / 10000.0));

    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().mode));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().gear));
    rc_publisher.publish(rc_joy);
  }

  tf::Matrix3x3 R_FRD2NED;
  tf::Quaternion q_FLU2ENU;

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q)
  {
    R_FRD2NED.setRotation(tf::Quaternion(vehicle->broadcast->getQuaternion().q1,
                                         vehicle->broadcast->getQuaternion().q2,
                                         vehicle->broadcast->getQuaternion().q3,
                                         vehicle->broadcast->getQuaternion().q0));
    tf::Matrix3x3 R_FLU2ENU = R_ENU2NED.transpose() * R_FRD2NED * R_FLU2FRD;
    R_FLU2ENU.getRotation(q_FLU2ENU);

    geometry_msgs::QuaternionStamped q;
    q.header.stamp = now_time;
    q.header.frame_id = "body_FLU";

    q.quaternion.w = q_FLU2ENU.getW();
    q.quaternion.x = q_FLU2ENU.getX();
    q.quaternion.y = q_FLU2ENU.getY();
    q.quaternion.z = q_FLU2ENU.getZ();

    attitude_publisher.publish(q);
  }

  if ( (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_Q) &&
       (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_W) &&
       (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_A))
  {
    sensor_msgs::Imu imu;

    imu.header.frame_id = "body_FLU";
    imu.header.stamp    = now_time;

    imu.linear_acceleration.x =  vehicle->broadcast->getAcceleration().x * gravity_const;
    imu.linear_acceleration.y = -vehicle->broadcast->getAcceleration().y * gravity_const;
    imu.linear_acceleration.z = -vehicle->broadcast->getAcceleration().z * gravity_const;

    imu.angular_velocity.x    =  vehicle->broadcast->getAngularRate().x;
    imu.angular_velocity.y    = -vehicle->broadcast->getAngularRate().y;
    imu.angular_velocity.z    = -vehicle->broadcast->getAngularRate().z;

    // Since the orientation is duplicated from attitude
    // at this point, q_FLU2ENU has already been updated
    imu.orientation.w = q_FLU2ENU.getW();
    imu.orientation.x = q_FLU2ENU.getX();
    imu.orientation.y = q_FLU2ENU.getY();
    imu.orientation.z = q_FLU2ENU.getZ();

    imu_publisher.publish(imu);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_POS)
  {
    DJI::OSDK::Telemetry::GlobalPosition global_pos =
      vehicle->broadcast->getGlobalPosition();
    std_msgs::UInt8 gps_health;
    gps_health.data = global_pos.health;
    gps_health_publisher.publish(gps_health);

    sensor_msgs::NavSatFix gps_pos;
    gps_pos.header.stamp    = now_time;
    gps_pos.header.frame_id = "/gps";
    gps_pos.latitude        = global_pos.latitude * 180 / C_PI;
    gps_pos.longitude       = global_pos.longitude * 180 / C_PI;
    gps_pos.altitude        = global_pos.altitude;
    gps_position_publisher.publish(gps_pos);
  }

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::HAS_V)
  {
    geometry_msgs::Vector3Stamped velocity;
    velocity.header.stamp    = now_time;
    velocity.header.frame_id = "ground_ENU";

    velocity.vector.x = vehicle->broadcast->getVelocity().y;
    velocity.vector.y = vehicle->broadcast->getVelocity().x;
    velocity.vector.z = vehicle->broadcast->getVelocity().z;
    velocity_publisher.publish(velocity);
  }

  if ( data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_BATTERY )
  {
    sensor_msgs::BatteryState msg_battery_state;
    msg_battery_state.capacity = NAN;
    msg_battery_state.voltage  = vehicle->broadcast->getBatteryInfo().voltage / 1000.0;
    msg_battery_state.current  = NAN;
    msg_battery_state.percentage = NAN;
    msg_battery_state.charge   = NAN;
    msg_battery_state.design_capacity = NAN;
    msg_battery_state.power_supply_health = msg_battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
    msg_battery_state.power_supply_status = msg_battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
    msg_battery_state.power_supply_technology = msg_battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
    msg_battery_state.present = (vehicle->broadcast->getBatteryInfo().voltage!=0);
    battery_state_publisher.publish(msg_battery_state);
  }

  if ( data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_STATUS )
  {
    Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
      vehicle->broadcast->getStatus().flight;

    std_msgs::UInt8 flight_status;
    flight_status.data = fs;
    flight_status_publisher.publish(flight_status);
  }

}

void
DJISDKNode::publish10HzData(Vehicle *vehicle, RecvContainer recvFrame,
                            DJI::OSDK::UserData userData)
{
  DJISDKNode *p = (DJISDKNode *)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_10HZ == *data );

  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();;

  if(p->align_time_with_FC)
  {
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  //TODO: publish gps detail data if needed
  Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type battery_info=
    vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();
  sensor_msgs::BatteryState msg_battery_state;
  msg_battery_state.capacity = NAN;
  msg_battery_state.voltage  = battery_info.voltage / 1000.0;
  msg_battery_state.current  = NAN;
  msg_battery_state.percentage = NAN;
  msg_battery_state.charge   = NAN;
  msg_battery_state.design_capacity = NAN;
  msg_battery_state.power_supply_health = msg_battery_state.POWER_SUPPLY_HEALTH_UNKNOWN;
  msg_battery_state.power_supply_status = msg_battery_state.POWER_SUPPLY_STATUS_UNKNOWN;
  msg_battery_state.power_supply_technology = msg_battery_state.POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  msg_battery_state.present = (battery_info.voltage!=0);
  p->battery_state_publisher.publish(msg_battery_state);

  return;
}


void
DJISDKNode::publish50HzData(Vehicle* vehicle, RecvContainer recvFrame,
                            DJI::OSDK::UserData userData)
{
  DJISDKNode* p = (DJISDKNode*)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_50HZ == *data );
  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time msg_time = ros::Time::now();;

  if(p->align_time_with_FC)
  {
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type fused_gps =
    vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();

  sensor_msgs::NavSatFix gps_pos;
  gps_pos.header.frame_id = "/gps";
  gps_pos.header.stamp    = msg_time;
  gps_pos.latitude        = fused_gps.latitude * 180 / C_PI;   //degree
  gps_pos.longitude       = fused_gps.longitude * 180 / C_PI;  //degree
  gps_pos.altitude        = fused_gps.altitude;                //meter
  p->gps_position_publisher.publish(gps_pos);

  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
    vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();

  std_msgs::UInt8 flight_status;
  flight_status.data = fs;
  p->flight_status_publisher.publish(flight_status);

  Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type v_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
  geometry_msgs::Vector3Stamped v;
  // v_FC has 2 fields, data and info. The latter contains the health

  /*!
   * note: We are now following REP 103 to use ENU for
   *       short-range Cartesian representations
   */
  v.header.frame_id = "ground_ENU";
  v.header.stamp = msg_time;
  v.vector.x = v_FC.data.y;  //x, y are swapped from NE to EN
  v.vector.y = v_FC.data.x;
  v.vector.z = v_FC.data.z; //z sign is already U
  p->velocity_publisher.publish(v);

  Telemetry::TypeMap<Telemetry::TOPIC_GPS_CONTROL_LEVEL>::type gps_ctrl_level=
    vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_CONTROL_LEVEL>();
  std_msgs::UInt8 msg_gps_ctrl_level;
  msg_gps_ctrl_level.data = gps_ctrl_level;
  p->gps_health_publisher.publish(msg_gps_ctrl_level);

  Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal_angle =
    vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();

  geometry_msgs::Vector3Stamped gimbal_angle_vec3;

  gimbal_angle_vec3.header.stamp = ros::Time::now();
  gimbal_angle_vec3.vector.x     = gimbal_angle.x;
  gimbal_angle_vec3.vector.y     = gimbal_angle.y;
  gimbal_angle_vec3.vector.z     = gimbal_angle.z;
  p->gimbal_angle_publisher.publish(gimbal_angle_vec3);


  // TODO: documentation to explain display mode
  Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type dm =
    vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

  std_msgs::UInt8 status_dm;
  status_dm.data = dm;
  p->displaymode_publisher.publish(status_dm);

  /**
    * NOTE: When using SBUS remote controllers, the rc.mode
    *       does not properly reflect the rc's input.
    *       Therefore the rc command uses broadcast info
    *       instead (for now).
    */
//  Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
//    vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();

  /********* RC Map (A3) *********
  *
  *       -10000  <--->  0      <---> 10000
  * MODE: API(F)  <---> ATTI(A) <--->  POS (P)
  *
  *        CH3 +10000                     CH1 +10000
  *               ^                              ^
  *               |                              |                   / -5000
  *    CH2        |                   CH0        |                  /
  *  -10000 <-----------> +10000    -10000 <-----------> +10000    H
  *               |                              |                  \
  *               |                              |                   \ -10000
  *               V                              V
  *            -10000                         -10000
  *
  *   In this code, before publish, RC is transformed to M100 style to be compatible with controller
  *****************************/

//  sensor_msgs::Joy rc_joy;
//  rc_joy.header.stamp    = msg_time;
//  rc_joy.header.frame_id = "rc";

//  rc_joy.axes.reserve(6);

//  rc_joy.axes.push_back(static_cast<float>(rc.roll     / 10000.0));
//  rc_joy.axes.push_back(static_cast<float>(rc.pitch    / 10000.0));
//  rc_joy.axes.push_back(static_cast<float>(rc.yaw      / 10000.0));
//  rc_joy.axes.push_back(static_cast<float>(rc.throttle / 10000.0));
//  rc_joy.axes.push_back(static_cast<float>(rc.mode*1.0));
//  rc_joy.axes.push_back(static_cast<float>(rc.gear*1.0));
//  p->rc_publisher.publish(rc_joy);
  short int data_enable_flag = vehicle->broadcast->getPassFlag();

  if (data_enable_flag & DataBroadcast::DATA_ENABLE_FLAG::A3_HAS_RC)
  {
    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp    = msg_time;
    rc_joy.header.frame_id = "rc";

    rc_joy.axes.reserve(6);
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().roll     / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().pitch    / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().yaw      / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().throttle / 10000.0));

    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().mode));
    rc_joy.axes.push_back(static_cast<float>(vehicle->broadcast->getRC().gear));
    p->rc_publisher.publish(rc_joy);
  }

}

void
DJISDKNode::publish100HzData(Vehicle *vehicle, RecvContainer recvFrame,
                                  DJI::OSDK::UserData userData)
{
  DJISDKNode *p = (DJISDKNode *)userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_100HZ == *data );
  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  ros::Time now_time = ros::Time::now();
  ros::Time msg_time = now_time;

  if(p->align_time_with_FC)
  {
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);

      // Check if time is drifting
      double dt = std::fabs((now_time - msg_time).toSec());
      if(dt > TIME_DIFF_ALERT)
      {
        static int cnt = 0;
        cnt++;
        ROS_WARN_THROTTLE(
                1.0, "[dji_sdk] ros::Time::now() - TickTime = %.0f ms [%d]", dt * 1000, cnt);
      }
    }
    else
    {
      return;
    }
  }

  Telemetry::TypeMap<Telemetry::TOPIC_QUATERNION>::type quat =
          vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
  geometry_msgs::QuaternionStamped q;

  /*!
   * note: We are now following REP 103 to use FLU for
   *       body frame. The quaternion is the rotation from
   *       body_FLU to ground_ENU
   */
  q.header.frame_id = "body_FLU";
  q.header.stamp    = msg_time;

  tf::Matrix3x3 R_FRD2NED(tf::Quaternion(quat.q1, quat.q2, quat.q3, quat.q0));
  tf::Matrix3x3 R_FLU2ENU = p->R_ENU2NED.transpose() * R_FRD2NED * p->R_FLU2FRD;
  tf::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);
  // @note this mapping is tested
  q.quaternion.w = q_FLU2ENU.getW();
  q.quaternion.x = q_FLU2ENU.getX();
  q.quaternion.y = q_FLU2ENU.getY();
  q.quaternion.z = q_FLU2ENU.getZ();
  p->attitude_publisher.publish(q);

  Telemetry::TypeMap<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>::type w_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();

  geometry_msgs::Vector3Stamped angular_rate;

  /*!
   * note: We are now following REP 103 to use FLU for
   *       body frame
   */
  angular_rate.header.frame_id = "body_FLU";
  angular_rate.header.stamp    = msg_time;

  angular_rate.vector.x        =  w_FC.x;
  angular_rate.vector.y        = -w_FC.y; //y,z sign are flipped from RD to LU
  angular_rate.vector.z        = -w_FC.z;
  p->angularRate_publisher.publish(angular_rate);

  Telemetry::TypeMap<Telemetry::TOPIC_ACCELERATION_GROUND>::type a_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
  geometry_msgs::Vector3Stamped acceleration;

  /*!
   * note: 1. We are now following REP 103 to use ENU for
   *       short-range Cartesian representations
   *
   *       2. TODO: This accel is in ground frame, which may
   *       cause confusion with the body-frame accel in imu message
   */

  acceleration.header.frame_id = "ground_ENU";
  acceleration.header.stamp    = msg_time;

  acceleration.vector.x        = a_FC.y;  //x, y are swapped from NE to EN
  acceleration.vector.y        = a_FC.x;
  acceleration.vector.z        = a_FC.z;  //z sign is already U
  p->acceleration_publisher.publish(acceleration);
}

void
DJISDKNode::publish400HzData(Vehicle *vehicle, RecvContainer recvFrame,
                                  DJI::OSDK::UserData userData)
{
  DJISDKNode *p = (DJISDKNode *) userData;

  uint8_t* data = recvFrame.recvData.raw_ack_array;
  ROS_ASSERT(DJISDKNode::PACKAGE_ID_400HZ == *data );

  data++;
  Telemetry::TimeStamp packageTimeStamp = * (reinterpret_cast<Telemetry::TimeStamp *>(data));

  Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type hardSync_FC =
    vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();

  ros::Time now_time = ros::Time::now();
  ros::Time msg_time = now_time;

  if(p->align_time_with_FC)
  {
    p->alignRosTimeWithFlightController(now_time, packageTimeStamp.time_ms);
    if(p->curr_align_state == ALIGNED)
    {
      msg_time = p->base_time + _TICK2ROSTIME(packageTimeStamp.time_ms);
    }
    else
    {
      return;
    }
  }

  sensor_msgs::Imu synced_imu;

  synced_imu.header.frame_id = "body_FLU";
  synced_imu.header.stamp    = msg_time;

  //y, z signs are flipped from RD to LU for rate and accel
  synced_imu.angular_velocity.x    =   hardSync_FC.w.x;
  synced_imu.angular_velocity.y    =  -hardSync_FC.w.y;
  synced_imu.angular_velocity.z    =  -hardSync_FC.w.z;

  synced_imu.linear_acceleration.x =   hardSync_FC.a.x * p->gravity_const;
  synced_imu.linear_acceleration.y =  -hardSync_FC.a.y * p->gravity_const;
  synced_imu.linear_acceleration.z =  -hardSync_FC.a.z * p->gravity_const;

  /*!
   * The quaternion is the rotation from body_FLU to ground_ENU.
   * Refer to:
   *   https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/imu_pub.cpp
   */
  tf::Matrix3x3 R_FRD2NED(tf::Quaternion(hardSync_FC.q.q1, hardSync_FC.q.q2,
                                         hardSync_FC.q.q3, hardSync_FC.q.q0));
  tf::Matrix3x3 R_FLU2ENU = p->R_ENU2NED.transpose() * R_FRD2NED * p->R_FLU2FRD;
  tf::Quaternion q_FLU2ENU;
  R_FLU2ENU.getRotation(q_FLU2ENU);

  synced_imu.orientation.w = q_FLU2ENU.getW();
  synced_imu.orientation.x = q_FLU2ENU.getX();
  synced_imu.orientation.y = q_FLU2ENU.getY();
  synced_imu.orientation.z = q_FLU2ENU.getZ();

  p->imu_publisher.publish(synced_imu);

  if (hardSync_FC.ts.flag == 1)
  {
    sensor_msgs::TimeReference trigTime;
    trigTime.header.stamp = msg_time;
    trigTime.time_ref     = now_time;
    trigTime.source       = "FC";

    p->trigger_publisher.publish(trigTime);
  }
}

/*!
 * @brief: The purpose of time alignment is to use the time received from flight
 *         controller to stamp all published ros messages. The reason is that the
 *         flight controller is running a real time system, while the ros time can
 *         be affected by OS scheduling depending on system load.
 */

void DJISDKNode::alignRosTimeWithFlightController(ros::Time now_time, uint32_t tick)
{
  if (curr_align_state == UNALIGNED)
  {
    base_time = now_time - _TICK2ROSTIME(tick);
    curr_align_state = ALIGNING;
    ROS_INFO("[dji_sdk] Start time alignment ...");
    return;
  }

  if (curr_align_state == ALIGNING)
  {
    static int aligned_count = 0;
    static int retry_count = 0;
    ROS_INFO_THROTTLE(1.0, "[dji_sdk] Aliging time...");

    double dt = std::fabs((now_time - (base_time + _TICK2ROSTIME(tick))).toSec());

    if(dt < TIME_DIFF_CHECK )
    {
      aligned_count++;
    }
    else if(aligned_count > 0)
    {
      base_time = now_time - _TICK2ROSTIME(tick);
      ROS_INFO("[dji_sdk] ***** Time difference out of bound after %d samples, retried %d times, dt=%.3f... *****",
               aligned_count, retry_count, dt);
      aligned_count = 0;
      retry_count++;
    }

    if(aligned_count > STABLE_ALIGNMENT_COUNT)
    {
      ROS_INFO("[dji_sdk] ***** Time alignment successful! *****");
      curr_align_state = ALIGNED;
    }

    return;
  }
}
