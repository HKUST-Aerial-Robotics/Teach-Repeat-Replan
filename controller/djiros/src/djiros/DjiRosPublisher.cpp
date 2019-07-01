//
// Created by ltb on 6/8/17.
//

#include <djiros/DjiRos.h>

bool DjiRos::initPublisher(ros::NodeHandle &nh) {
  rc_publisher = nh.advertise<sensor_msgs::Joy>("rc", 10);

  imu_publisher = nh.advertise<sensor_msgs::Imu>("imu", 10);

  flight_status_publisher = nh.advertise<std_msgs::UInt8>("flight_status", 10);

  gps_health_publisher = nh.advertise<std_msgs::UInt8>("gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);

  velocity_publisher = nh.advertise<geometry_msgs::Vector3Stamped>("velo", 10);

  from_mobile_data_publisher = nh.advertise<dji_sdk::MobileData>("from_mobile_data", 10);

  gimbal_angle_publisher = nh.advertise<geometry_msgs::Vector3Stamped>("gimbal_angle", 10);

//    ACK::ErrorCode broadcast_set_freq_ack;

  if (telemetry_from_fc == USE_BROADCAST) {
    ROS_ASSERT_MSG(false, "Broadcast not supported!");
    //        ROS_INFO("Hardware or firmware only support data broadcast!");
    //        // default freq 50Hz
    //        broadcast_set_freq_ack =
    //                vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);
    //
    //        if (ACK::getError(broadcast_set_freq_ack))
    //        {
    //            ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
    //            return false;
    //        }
    //        // register a callback function whenever a broadcast data is in
    //        vehicle->broadcast->setUserBroadcastCallback(
    //                &DJISDKNode::SDKBroadcastCallback, this);
  } else if (telemetry_from_fc == USE_SUBSCRIBE) {
    ROS_INFO("Hardware and firmware support data subscription!");

    // Extra topics that is only available from subscription
    displaymode_publisher = nh.advertise<std_msgs::UInt8>("display_mode", 10);

    if (!initDataSubscribeFromFC()) {
      return false;
    }
  }
  vehicle->moc->setFromMSDKCallback(&DJISDKNode::SDKfromMobileDataCallback, this);
  return true;
};

bool DjiRos::initDataSubscribeFromFC() {
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack)) {
    return false;
  }

  // 400 Hz package from FC
  Telemetry::TopicName topicList400Hz[] = {Telemetry::TOPIC_HARD_SYNC};
  int nTopic400Hz = sizeof(topicList400Hz) / sizeof(topicList400Hz[0]);
  int packageID400Hz = 0;
  if (vehicle->subscribe->initPackageFromTopicList(
      packageID400Hz, nTopic400Hz, topicList400Hz, 0, 400)) {
    ack = vehicle->subscribe->startPackage(packageID400Hz, WAIT_TIMEOUT);
    if (ACK::getError(ack)) {
      vehicle->subscribe->removePackage(packageID400Hz, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 400Hz mpackage");
      return false;
    } else {
      vehicle->subscribe->registerUserPackageUnpackCallback(
          packageID400Hz, DjiRos::onReceive400HzData, this);
    }
  }

  // 50 Hz package from FC
  Telemetry::TopicName topicList50Hz[] = {Telemetry::TOPIC_VELOCITY,
                                          Telemetry::TOPIC_GPS_FUSED,
                                          Telemetry::TOPIC_HEIGHT_FUSION,
                                          Telemetry::TOPIC_STATUS_FLIGHT,
                                          Telemetry::TOPIC_STATUS_DISPLAYMODE,
//                                            Telemetry::TOPIC_GPS_DATE,
//                                            Telemetry::TOPIC_GPS_TIME,
//                                            Telemetry::TOPIC_GPS_POSITION,
//                                            Telemetry::TOPIC_GPS_VELOCITY,
//                                            Telemetry::TOPIC_GPS_DETAILS,
                                          Telemetry::TOPIC_GIMBAL_ANGLES,
//                                            Telemetry::TOPIC_GIMBAL_STATUS,
                                          Telemetry::TOPIC_RC};
  int nTopic50Hz = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  int packageID50Hz = 1;

  if (vehicle->subscribe->initPackageFromTopicList(
      packageID50Hz, nTopic50Hz, topicList50Hz, 0, 50)) {
    ack = vehicle->subscribe->startPackage(packageID50Hz, WAIT_TIMEOUT);
    if (ACK::getError(ack)) {
      vehicle->subscribe->removePackage(packageID50Hz, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz mpackage");
      return false;
    } else {
      vehicle->subscribe->registerUserPackageUnpackCallback(
          packageID50Hz, DjiRos::onReceive50HzData, (UserData) this);
    }
  }
  return true;
}

template<typename T>
bool validate_answer(const T &ans) {
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&ans);
  bool invalid = true;

  for (size_t k = 0; k < sizeof(T); ++k) {
    invalid &= (p[k] == 0xFF);
  }

  return !invalid;
}

void DjiRos::onReceive50HzData(Vehicle *vehicle,
                               RecvContainer recvFrame,
                               DJI::OSDK::UserData userData) {
  DjiRos *p = (DjiRos *) userData;

  ros::Time msg_stamp = p->aligner.acquire_latest_stamp();

  do {   // Velocity
    Telemetry::TypeMap<Telemetry::TOPIC_VELOCITY>::type vel =
        vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
    geometry_msgs::Vector3Stamped velo_msg;

    if (!validate_answer(vel)) break;

    if (vel.info.health) {
      velo_msg.header.stamp = msg_stamp;
      velo_msg.header.frame_id = std::string("NED");
      velo_msg.vector.x = static_cast<double>(vel.data.x);
      velo_msg.vector.y = static_cast<double>(vel.data.y);
      velo_msg.vector.z = static_cast<double>(-vel.data.z);
      p->velocity_publisher.publish(velo_msg);
    }
  } while (0);

  do {   // GPS
    Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type latlong =
        vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
    Telemetry::TypeMap<Telemetry::TOPIC_HEIGHT_FUSION>::type height =
        vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();

    if (!validate_answer(latlong) || !validate_answer(height)) break;

    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.frame_id = "NED";
    gps_msg.header.stamp = msg_stamp;
    gps_msg.latitude = latlong.latitude * 180.0 / M_PI;
    gps_msg.longitude = latlong.longitude * 180.0 / M_PI;
    gps_msg.altitude = height;
    gps_msg.status.status = latlong.visibleSatelliteNumber;
    p->gps_position_publisher.publish(gps_msg);

  } while (0);

  do { // Flight Status
    Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type fs =
        vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();

    if (!validate_answer(fs)) break;

    std_msgs::UInt8 flight_status;
    flight_status.data = fs;
    p->flight_status_publisher.publish(flight_status);
  } while (0);

  do { // Gimbal
    Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal_angle =
        vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();

    if (!validate_answer(gimbal_angle)) break;

    geometry_msgs::Vector3Stamped gimbal_angle_vec3;
    gimbal_angle_vec3.header.stamp = msg_stamp;
    gimbal_angle_vec3.vector.x = gimbal_angle.x;
    gimbal_angle_vec3.vector.y = gimbal_angle.y;
    gimbal_angle_vec3.vector.z = gimbal_angle.z;
    p->gimbal_angle_publisher.publish(gimbal_angle_vec3);
  } while (0);

  do { // DisplayMode
    Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type dm =
        vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();

    if (!validate_answer(dm)) break;

    std_msgs::UInt8 status_dm;
    status_dm.data = dm;
    p->displaymode_publisher.publish(status_dm);
  } while (0);

  do { // Remote Controller
    Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc =
        vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();

    if (!validate_answer(rc)) break;

    sensor_msgs::Joy rc_joy;
    rc_joy.header.stamp = msg_stamp;
    rc_joy.header.frame_id = std::string("rc");

    rc_joy.axes.reserve(6);
    rc_joy.axes.push_back(static_cast<float>(rc.roll / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.pitch / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.yaw / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.throttle / 10000.0));
    rc_joy.axes.push_back(static_cast<float>(rc.mode / 10000.0));
    rc_joy.axes.push_back(static_cast<float>((-rc.gear - 15000) / 10000.0));
    p->rc_publisher.publish(rc_joy);

    p->api_trigger.setValue(rc.mode);

  } while (0);
}

void DjiRos::onReceive400HzData(Vehicle *vehicle,
                                RecvContainer recvFrame,
                                DJI::OSDK::UserData userData) {
  DjiRos *p = (DjiRos *) userData;

  Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type hs_data =
      vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();

  sensor_msgs::Imu imu_msg;

  bool aligned = p->aligner.acquire_stamp(imu_msg.header.stamp, hs_data.ts.time2p5ms);

  if (!aligned)
    return;

  imu_msg.header.frame_id = std::string("FLU");

  // transform to ROS REP 103 Convention
  Eigen::Quaterniond q_fc(hs_data.q.q0, hs_data.q.q1, hs_data.q.q2, hs_data.q.q3);
  Eigen::Quaterniond q_ros(p->ros_R_fc * q_fc.toRotationMatrix() * p->ros_R_fc.transpose());

  imu_msg.orientation.w = q_ros.w();
  imu_msg.orientation.x = q_ros.x();
  imu_msg.orientation.y = q_ros.y();
  imu_msg.orientation.z = q_ros.z();

  imu_msg.angular_velocity.x = hs_data.w.x;
  imu_msg.angular_velocity.y = -hs_data.w.y;
  imu_msg.angular_velocity.z = -hs_data.w.z;

  imu_msg.linear_acceleration.x = hs_data.a.x * p->gravity;
  imu_msg.linear_acceleration.y = -hs_data.a.y * p->gravity;
  imu_msg.linear_acceleration.z = -hs_data.a.z * p->gravity;

  p->imu_publisher.publish(imu_msg);

  if (hs_data.ts.flag) {
    //    ROS_DEBUG("sdk recv sync:%d #%d @ %d.%d",
    //              bc_data.timeStamp.syncFlag,
    //              m_hwsync_ack_count,
    //              msg_stamp.sec, msg_stamp.nsec);

    if (p->m_hwsync.get()) {
      auto hwsync = p->m_hwsync;
      std::lock_guard<std::mutex> lg(hwsync->ack_mutex);
      hwsync->ack_queue.emplace(imu_msg.header.stamp, p->m_hwsync_ack_count);
    }

    p->m_hwsync_ack_count++;
  }

  //  ROS_INFO("Sync: f[%d] idx[%d] tick[%d]", hs_data.ts.flag, hs_data.ts.index,
  //  hs_data.ts.time2p5ms);
}
