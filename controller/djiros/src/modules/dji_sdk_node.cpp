/** @file dji_sdk_node.cpp
 *  @version 3.3
 *  @date May, 2017
 *
 *  @brief
 *  Implementation of the initialization functions of DJISDKNode
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

using namespace DJI::OSDK;

DJISDKNode::DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : telemetry_from_fc(USE_BROADCAST),
    R_FLU2FRD(tf::Matrix3x3(1,  0,  0, 0, -1,  0, 0,  0, -1)),
    R_ENU2NED(tf::Matrix3x3(0,  1,  0, 1,  0,  0, 0,  0, -1)),
    curr_align_state(UNALIGNED)
{
  nh_private.param("serial_name",   serial_device, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate",     baud_rate, 921600);
  nh_private.param("app_id",        app_id,    123456);
  nh_private.param("app_version",   app_version, 1);
  nh_private.param("enc_key",       enc_key, std::string("abcd1234"));
  nh_private.param("uart_or_usb",   uart_or_usb, 0); // chosse uart as default
  nh_private.param("drone_version", drone_version, std::string("M100")); // choose M100 as default
  nh_private.param("gravity_const", gravity_const, 9.801);
  nh_private.param("align_time",    align_time_with_FC, true);
  nh_private.param("use_broadcast", user_select_BC, false);

  // @todo need some error handling for init functions
  //! @note parsing launch file to get environment parameters
  if (!initVehicle(nh_private))
  {
    ROS_ERROR("Vehicle initialization failed");
  }

  else
  {
    if (!initServices(nh))
    {
      ROS_ERROR("initServices failed");
    }

    if (!initFlightControl(nh))
    {
      ROS_ERROR("initFlightControl failed");
    }

    if (!initSubscriber(nh))
    {
      ROS_ERROR("initSubscriber failed");
    }

    if (!initPublisher(nh))
    {
      ROS_ERROR("initPublisher failed");
    }
  }
}

DJISDKNode::~DJISDKNode()
{
  cleanUpSubscribeFromFC();
  if (vehicle)
  {
    delete vehicle;
  }
}

bool
DJISDKNode::initVehicle(ros::NodeHandle& nh_private)
{
  bool threadSupport = true;
  // LinuxSerialDevice* linuxSerialDevice = new LinuxSerialDevice(serial_device.c_str(),baud_rate);
  // linuxSerialDevice->init();
  // bool setupStatus = validateSerialDevice(linuxSerialDevice);
  // if(!setupStatus)
  // {
  //   delete (linuxSerialDevice);
  //   return false;
  // }
  // else
  // {
  //   delete(linuxSerialDevice);
  // }

  //! @note currently does not work without thread support
  vehicle = new Vehicle(serial_device.c_str(), baud_rate, threadSupport);

  if (ACK::getError(this->activate(this->app_id, this->enc_key)))
  {
    ROS_ERROR("drone activation error");
    return false;
  }
  ROS_INFO("drone activated");

  if(vehicle->getFwVersion() > 0 && vehicle->getFwVersion() < mandatoryVersionBase)
  {
    return false;
  }

  for (int i = 0; i < MAX_SUBSCRIBE_PACKAGES; i++)
    vehicle->subscribe->removePackage(i, WAIT_TIMEOUT);

  /*!
   * @note activate the drone for the user at the beginning
   *        user can also call it as a service
   *        this has been tested by giving wrong appID in launch file
   */
  

  if (NULL != vehicle->subscribe && (!user_select_BC))
  {
    telemetry_from_fc = USE_SUBSCRIBE;
  }

  return true;
}

// clang-format off
bool DJISDKNode::initServices(ros::NodeHandle& nh) {
  drone_activation_server   = nh.advertiseService("dji_sdk/activation",                     &DJISDKNode::droneActivationCallback,        this);
  drone_arm_server          = nh.advertiseService("dji_sdk/drone_arm_control",              &DJISDKNode::droneArmCallback,               this);
  drone_task_server         = nh.advertiseService("dji_sdk/drone_task_control",             &DJISDKNode::droneTaskCallback,              this);
  sdk_ctrlAuthority_server  = nh.advertiseService("dji_sdk/sdk_control_authority",          &DJISDKNode::sdkCtrlAuthorityCallback,       this);
  camera_action_server      = nh.advertiseService("dji_sdk/camera_action",                  &DJISDKNode::cameraActionCallback,           this);
  mfio_config_server        = nh.advertiseService("dji_sdk/mfio_config",                    &DJISDKNode::MFIOConfigCallback,             this);
  mfio_set_value_server     = nh.advertiseService("dji_sdk/mfio_set_value",                 &DJISDKNode::MFIOSetValueCallback,           this);
  waypoint_upload_server    = nh.advertiseService("dji_sdk/mission_waypoint_upload",        &DJISDKNode::missionWpUploadCallback,        this);
  waypoint_action_server    = nh.advertiseService("dji_sdk/mission_waypoint_action",        &DJISDKNode::missionWpActionCallback,        this);
  waypoint_getInfo_server   = nh.advertiseService("dji_sdk/mission_waypoint_getInfo",       &DJISDKNode::missionWpGetInfoCallback,       this);
  waypoint_getSpeed_server  = nh.advertiseService("dji_sdk/mission_waypoint_getSpeed",      &DJISDKNode::missionWpGetSpeedCallback,      this);
  waypoint_setSpeed_server  = nh.advertiseService("dji_sdk/mission_waypoint_setSpeed",      &DJISDKNode::missionWpSetSpeedCallback,      this);
  hotpoint_upload_server    = nh.advertiseService("dji_sdk/mission_hotpoint_upload",        &DJISDKNode::missionHpUploadCallback,        this);
  hotpoint_action_server    = nh.advertiseService("dji_sdk/mission_hotpoint_action",        &DJISDKNode::missionHpActionCallback,        this);
  hotpoint_getInfo_server   = nh.advertiseService("dji_sdk/mission_hotpoint_getInfo",       &DJISDKNode::missionHpGetInfoCallback,       this);
  hotpoint_setSpeed_server  = nh.advertiseService("dji_sdk/mission_hotpoint_updateYawRate", &DJISDKNode::missionHpUpdateYawRateCallback, this);
  hotpoint_resetYaw_server  = nh.advertiseService("dji_sdk/mission_hotpoint_resetYaw",      &DJISDKNode::missionHpResetYawCallback,      this);
  hotpoint_setRadius_server = nh.advertiseService("dji_sdk/mission_hotpoint_updateRadius",  &DJISDKNode::missionHpUpdateRadiusCallback,  this);
  mission_status_server     = nh.advertiseService("dji_sdkmission_status",                  &DJISDKNode::missionStatusCallback,          this);
  send_to_mobile_server     = nh.advertiseService("dji_sdk/send_data_to_mobile",            &DJISDKNode::sendToMobileCallback,           this);
  set_hardsync_server       = nh.advertiseService("dji_sdk/set_hardsyc",                    &DJISDKNode::setHardsyncCallback,            this);
  return true;
}
// clang-format on

bool
DJISDKNode::initFlightControl(ros::NodeHandle& nh)
{
  flight_control_sub = nh.subscribe<sensor_msgs::Joy>(
    "dji_sdk/flight_control_setpoint_generic", 10, 
    &DJISDKNode::flightControlSetpointCallback,   this);

  flight_control_position_yaw_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_ENUposition_yaw", 10,
      &DJISDKNode::flightControlPxPyPzYawCallback, this);

  flight_control_velocity_yawrate_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", 10,
      &DJISDKNode::flightControlVxVyVzYawrateCallback, this);

  flight_control_rollpitch_yawrate_vertpos_sub =
    nh.subscribe<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_rollpitch_yawrate_zposition", 10,
      &DJISDKNode::flightControlRollPitchPzYawrateCallback, this);

  return true;
}

ACK::ErrorCode
DJISDKNode::activate(int l_app_id, std::string l_enc_key)
{
  usleep(1000000);
  Vehicle::ActivateData testActivateData;
  char                  app_key[65];
  testActivateData.encKey = app_key;
  strcpy(testActivateData.encKey, l_enc_key.c_str());
  testActivateData.ID = l_app_id;

  ROS_DEBUG("called vehicle->activate(&testActivateData, WAIT_TIMEOUT)");
  return vehicle->activate(&testActivateData, WAIT_TIMEOUT);
}

bool
DJISDKNode::initSubscriber(ros::NodeHandle& nh)
{
  gimbal_angle_cmd_subscriber = nh.subscribe<dji_sdk::Gimbal>(
    "dji_sdk/gimbal_angle_cmd", 10, &DJISDKNode::gimbalAngleCtrlCallback, this);
  gimbal_speed_cmd_subscriber = nh.subscribe<geometry_msgs::Vector3Stamped>(
    "dji_sdk/gimbal_speed_cmd", 10, &DJISDKNode::gimbalSpeedCtrlCallback, this);
  return true;
}

bool
DJISDKNode::initPublisher(ros::NodeHandle& nh)
{
  rc_publisher = nh.advertise<sensor_msgs::Joy>("dji_sdk/rc", 10);

  attitude_publisher =
    nh.advertise<geometry_msgs::QuaternionStamped>("dji_sdk/attitude", 10);

  battery_state_publisher =
    nh.advertise<sensor_msgs::BatteryState>("dji_sdk/battery_state",10);

  /*!
   * - Fused attitude (duplicated from attitude topic)
   * - Raw linear acceleration (body frame: FLU, m/s^2)
   *       Z value is +9.8 when placed on level ground statically
   * - Raw angular velocity (body frame: FLU, rad/s^2)
   */
  imu_publisher = nh.advertise<sensor_msgs::Imu>("dji_sdk/imu", 10);

  //TODO: documentation
  flight_status_publisher =
    nh.advertise<std_msgs::UInt8>("dji_sdk/flight_status", 10);

  //TODO: documentation
  gps_health_publisher =
    nh.advertise<std_msgs::UInt8>("dji_sdk/gps_health", 10);

  /*!
   * NavSatFix specs:
   *   Latitude [degrees]. Positive is north of equator; negative is south.
   *   Longitude [degrees]. Positive is east of prime meridian; negative is
   * west.
   *   Altitude [m]. Positive is above the WGS 84 ellipsoid
   */
  gps_position_publisher =
    nh.advertise<sensor_msgs::NavSatFix>("dji_sdk/gps_position", 10);

  velocity_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/velocity", 10);

  from_mobile_data_publisher =
    nh.advertise<dji_sdk::MobileData>("dji_sdk/from_mobile_data", 10);

  // TODO: documentation and proper frame id
  gimbal_angle_publisher =
    nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/gimbal_angle", 10);

  if (telemetry_from_fc == USE_BROADCAST)
  {
    ACK::ErrorCode broadcast_set_freq_ack;
    ROS_INFO("Use legacy data broadcast to get telemetry data!");
    // default freq 50Hz
    broadcast_set_freq_ack =
      vehicle->broadcast->setBroadcastFreqDefaults(WAIT_TIMEOUT);

    if (ACK::getError(broadcast_set_freq_ack))
    {
      ACK::getErrorCodeMessage(broadcast_set_freq_ack, __func__);
      return false;
    }
    // register a callback function whenever a broadcast data is in
    vehicle->broadcast->setUserBroadcastCallback(
      &DJISDKNode::SDKBroadcastCallback, this);
  }
  else if (telemetry_from_fc == USE_SUBSCRIBE)
  {
    ROS_INFO("Use data subscription to get telemetry data!");
    if(!align_time_with_FC)
    {
      ROS_INFO("align_time_with_FC set to false. We will use ros time to time stamp messages!");
    }
    else
    {
      ROS_INFO("align_time_with_FC set to false. We will time stamp messages based on flight controller time!");
    }

    // Extra topics that is only available from subscription

    // Details can be found in DisplayMode enum in dji_sdk.h
    displaymode_publisher =
      nh.advertise<std_msgs::UInt8>("dji_sdk/display_mode", 10);

    angularRate_publisher =
      nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/angular_velocity_fused", 10);

    acceleration_publisher =
      nh.advertise<geometry_msgs::Vector3Stamped>("dji_sdk/acceleration_ground_fused", 10);

    trigger_publisher = nh.advertise<sensor_msgs::TimeReference>("dji_sdk/trigger_time", 10);

    if (!initDataSubscribeFromFC())
    {
      return false;
    }
  }
  vehicle->moc->setFromMSDKCallback(&DJISDKNode::SDKfromMobileDataCallback,
                                    this);
  return true;
}

bool
DJISDKNode::initDataSubscribeFromFC()
{
  ACK::ErrorCode ack = vehicle->subscribe->verify(WAIT_TIMEOUT);
  if (ACK::getError(ack))
  {
    return false;
  }

  // 100 Hz package from FC
  Telemetry::TopicName topicList100Hz[] = {
    Telemetry::TOPIC_QUATERNION,
    Telemetry::TOPIC_ACCELERATION_GROUND,
    Telemetry::TOPIC_ANGULAR_RATE_FUSIONED
  };
  int nTopic100Hz    = sizeof(topicList100Hz) / sizeof(topicList100Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_100HZ, nTopic100Hz,
                                                   topicList100Hz, 1, 100))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_100HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 100Hz mpackage");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_100HZ, publish100HzData, this);
    }
  }

  // 50 Hz package from FC
  Telemetry::TopicName topicList50Hz[] = {
    Telemetry::TOPIC_GPS_FUSED,
    Telemetry::TOPIC_STATUS_FLIGHT,
    Telemetry::TOPIC_STATUS_DISPLAYMODE,
    Telemetry::TOPIC_GIMBAL_ANGLES,
    Telemetry::TOPIC_GIMBAL_STATUS,
    Telemetry::TOPIC_RC,
    Telemetry::TOPIC_VELOCITY,
    Telemetry::TOPIC_GPS_CONTROL_LEVEL
  };
  int nTopic50Hz    = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_50HZ, nTopic50Hz,
                                                   topicList50Hz, 1, 50))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
    if (ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_50HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 50Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(
              PACKAGE_ID_50HZ, publish50HzData, (UserData) this);
    }
  }

  // 10 Hz package from FC
  Telemetry::TopicName topicList10Hz[] = {
          Telemetry::TOPIC_GPS_DATE,
          Telemetry::TOPIC_GPS_TIME,
          Telemetry::TOPIC_GPS_POSITION,
          Telemetry::TOPIC_GPS_VELOCITY,
          Telemetry::TOPIC_GPS_DETAILS,
          Telemetry::TOPIC_BATTERY_INFO
  };
  int nTopic10Hz = sizeof(topicList10Hz) /sizeof(topicList10Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_10HZ, nTopic10Hz,
                                                   topicList10Hz, 1, 10))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_10HZ, WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_10HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 10Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(PACKAGE_ID_10HZ, publish10HzData, this);
    }
  }

  // 400 Hz data from FC
  Telemetry::TopicName topicList400Hz[] = {
          Telemetry::TOPIC_HARD_SYNC
  };
  int nTopic400Hz = sizeof(topicList400Hz) / sizeof(topicList400Hz[0]);
  if (vehicle->subscribe->initPackageFromTopicList(PACKAGE_ID_400HZ, nTopic400Hz,
                                                   topicList400Hz, 1, 400))
  {
    ack = vehicle->subscribe->startPackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
    if(ACK::getError(ack))
    {
      vehicle->subscribe->removePackage(PACKAGE_ID_400HZ, WAIT_TIMEOUT);
      ROS_ERROR("Failed to start 400Hz package");
      return false;
    }
    else
    {
      vehicle->subscribe->registerUserPackageUnpackCallback(PACKAGE_ID_400HZ, publish400HzData, this);
    }
  }

  return true;
}

void
DJISDKNode::cleanUpSubscribeFromFC()
{
  vehicle->subscribe->removePackage(0, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(1, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(2, WAIT_TIMEOUT);
  vehicle->subscribe->removePackage(3, WAIT_TIMEOUT);
}

bool DJISDKNode::validateSerialDevice(LinuxSerialDevice* serialDevice)
{
  static const int BUFFER_SIZE = 2048;
  //! Check the serial channel for data
  uint8_t buf[BUFFER_SIZE];
  if (!serialDevice->setSerialPureTimedRead()) {
    ROS_ERROR("Failed to set up port for timed read.\n"
               "This usually means the serial port is not correctly set up; \n"
               "however on a small number of machines this error can come up\n"
               "even when the port is correctly set up. If you are absolutely certain\n"
               "your connections are okay, try commenting out L77-L89 in file dji_sdk_node.cpp\n"
               "and build again.\n");
    return (false);
  };
  usleep(100000);
  if(serialDevice->serialRead(buf, BUFFER_SIZE))
  {
    ROS_INFO("Succeeded to read from serial device");
  }
  else
  {
    ROS_ERROR("Failed to read from serial device. The Onboard SDK is not communicating with your drone.");
    return (false);
  }

  // All the tests passed and the serial device is properly set up
  serialDevice->unsetSerialPureTimedRead();
  return (true);
}
