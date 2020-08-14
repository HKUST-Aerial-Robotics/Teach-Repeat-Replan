#include "cascadecontrollernode.hpp"

#ifdef HAVE_DJIOSDK
#define HAVE_DJIOSDK_PRIVATE
#include <djiosdk/dji_control.hpp>
#include <djiosdk/dji_status.hpp>
#endif

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <uav_utils/geometry_utils.h>

#include "accelerationcontrol.hpp"
#include "position.hpp"
#include "velocity.hpp"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(CascadeControllerNode, nodelet::Nodelet);

CascadeControllerNode::CascadeControllerNode()
  : ctrl(new mocka::CascadeController())
  , resetQuaternion(true)
  , fcQuaternion(0, 0, 0, 0)
  , correctionThrust(0)
  , lastIntergrade(0)
{
  correctionMutex = PTHREAD_MUTEX_INITIALIZER;
  assert(fcQuaternion.x() == 0);
  assert(fcQuaternion.y() == 0);
  assert(fcQuaternion.z() == 0);
  assert(fcQuaternion.w() == 0);
}

CascadeControllerNode::~CascadeControllerNode()
{
  delete ctrl;
}

Eigen::Matrix3d
CascadeControllerNode::getMatrixParam(ros::NodeHandle& nh,
                                      Eigen::Matrix3d& value_default,
                                      std::string      name)
{
  std::vector<double> param;
  Eigen::Matrix3d     memory;
  if (nh.getParam(name, param))
  {
    memory = Eigen::Matrix3d(param.data());
    std::cout << name << " : \n" << memory.transpose() << std::endl;
    return memory.transpose();
  }
  else
  {
    ROS_WARN("%s init under default parameter", name.c_str());
    return value_default;
  }
}

void
CascadeControllerNode::update_position(const Eigen::Vector3d& data)
{
  auto now = std::chrono::system_clock::now();
  DT   ms  = now - lastPositionTime;
  auto dpe = (data - lastPosition) / ms.count();

  lastPosition     = data;
  lastPositionTime = now;

  alignPV.pushObservation(dpe);
  unusedPosition.push(data);
}

void
CascadeControllerNode::update_velocity(const Eigen::Vector3d& data)
{
  auto now = std::chrono::system_clock::now();
  DT   ms  = now - lastVelocityTime;
  auto dve = (data - lastVelocity) / ms.count();

  lastVelocity     = data;
  lastVelocityTime = now;

  alignPV.pushMeasuerment(data);
  alignVA.pushObservation(dve);
  unusedVelocity.push(data);
}

void
CascadeControllerNode::update_acceleration(const Eigen::Vector3d& data)
{

  alignVA.pushMeasuerment(data);
  unusedAcceleration.push(data);
}

bool
CascadeControllerNode::feed_position()
{
  Eigen::Vector3d data;
  data.setZero();

  int size = unusedPosition.size();

  while (!unusedPosition.empty())
  {
    data += unusedPosition.front();
    unusedPosition.pop();
  }

  data = data / static_cast<double>(size);

  if (std::isnan(data.norm()))
    return false;

  ctrl->setFeedPosition(data);
  return true;
}

bool
CascadeControllerNode::feed_velocity()
{

  Eigen::Vector3d data;
  data.setZero();

  int size = unusedVelocity.size();

  while (!unusedVelocity.empty())
  {
    data += unusedVelocity.front();
    unusedVelocity.pop();
  }

  data = data / static_cast<double>(size);

  if (std::isnan(data.norm()))
    return false;

  ctrl->setFeedVelocity(data);
  return true;
}

bool
CascadeControllerNode::feed_acceleration()
{
  Eigen::Vector3d data;
  data.setZero();

  int size = unusedAcceleration.size();

  while (!unusedAcceleration.empty())
  {
    data += unusedAcceleration.front();
    unusedAcceleration.pop();
  }

  data = data / static_cast<double>(size);

  if (std::isnan(data.norm()))
    return false;

  ctrl->setFeedAcceleration(data);
  return true;
}

void
CascadeControllerNode::onInit()
{
  //! @todo implement an API for these data segments
  enable = true;
  //! @note dirty below
  insky        = false;
  firstObserve = true;
  dirtyCounter = 0;

  ros::NodeHandle nh(getPrivateNodeHandle());

  std::string quadrotor_name;
  nh.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  nh.param("gains/rot/x", kR[0], 1.5);
  nh.param("gains/rot/y", kR[1], 1.5);
  nh.param("gains/rot/z", kR[2], 1.0);
  nh.param("gains/ang/x", kOm[0], 0.13);
  nh.param("gains/ang/y", kOm[1], 0.13);
  nh.param("gains/ang/z", kOm[2], 0.1);

  nh.param("corrections/z", corrections[0], 0.0);
  nh.param("corrections/r", corrections[1], 0.0);
  nh.param("corrections/p", corrections[2], 0.0);

  nh.param("mass", mass, 0.98);
  nh.param("max_thrust", maxThrust, 52.92 / mass);

  nh.param("simulation", isSimulation, true);

  frame_id = "/" + quadrotor_name;

  // publishers
  // clang-format off
  manual_command_pub    = nh.advertise<sensor_msgs::Joy>("manual_command", 2);
  attitude_command_pub  = nh.advertise<sensor_msgs::Joy>("attitude_command", 2);
  palstance_command_pub = nh.advertise<sensor_msgs::Joy>("palstance_command", 2);
  translation_error_pub = nh.advertise<geometry_msgs::Point>("translation_error", 2);
  so3_command_pub       = nh.advertise<quadrotor_msgs::SO3Command>("so3_command", 2);

  // clang-format off
  position_command_sub = nh.subscribe("position_command", 10, &CascadeControllerNode::position_command_callback, this, ros::TransportHints().tcpNoDelay());
  odometry_command_sub = nh.subscribe("odom_correction" , 10, &CascadeControllerNode::odom_correction_callback,  this, ros::TransportHints().tcpNoDelay());

  flightstatus_sub = nh.subscribe("sdk/flight_status", 10, &CascadeControllerNode::flight_status_callback, this, ros::TransportHints().tcpNoDelay());

  // subscribers
  odometry_sub      = nh.subscribe("odom",      10, &CascadeControllerNode::odom_callback,          this, ros::TransportHints().tcpNoDelay());
  imu_acc_world_sub = nh.subscribe("imu_world", 10, &CascadeControllerNode::imu_acc_world_callback, this, ros::TransportHints().tcpNoDelay());
  imu_sub           = nh.subscribe("imu",       10, &CascadeControllerNode::imu_callback,           this, ros::TransportHints().tcpNoDelay());
  view_sub          = nh.subscribe("view",      10, &CascadeControllerNode::view_callback,          this, ros::TransportHints().tcpNoDelay());
  thrust_sub        = nh.subscribe("thrust",    10, &CascadeControllerNode::thrust_callback,        this, ros::TransportHints().tcpNoDelay());
  gain_sub          = nh.subscribe("gain",      10, &CascadeControllerNode::gain_callback,          this, ros::TransportHints().tcpNoDelay());
  // clang-format on

  tempViewpoint = Eigen::Vector3d(1, 0, 0);

  Eigen::Matrix3d value_default;

  value_default.setZero();

  Eigen::Matrix3d posCoeff;
  Eigen::Matrix3d velCoeff;
  Eigen::Matrix3d accCoeff;

  Eigen::Matrix3d posLimit;
  Eigen::Matrix3d velLimit;
  Eigen::Matrix3d accLimit;

  posCoeff = getMatrixParam(nh, value_default, "position_coeff");
  velCoeff = getMatrixParam(nh, value_default, "velocity_coeff");
  accCoeff = getMatrixParam(nh, value_default, "acceleration_coeff");

  posLimit = getMatrixParam(nh, value_default, "position_limit");
  velLimit = getMatrixParam(nh, value_default, "velocity_limit");
  accLimit = getMatrixParam(nh, value_default, "acceleration_limit");

  ctrl->getControllerPos()->setCoeff(posCoeff);
  ctrl->getControllerVel()->setCoeff(velCoeff);
  ctrl->getControllerAcc()->setCoeff(accCoeff);

  ctrl->getControllerPos()->setLimitation(posLimit);
  ctrl->getControllerVel()->setLimitation(velLimit);
  ctrl->getControllerAcc()->setLimitation(accLimit);

  //! @todo dynamical parameter
  ctrl->getMixerVelocity()->setThreshold(Eigen::Vector3d(1, 1, 1));
  ctrl->getMixerAcceleration()->setThreshold(Eigen::Vector3d(1, 1, 1));

  auto now = std::chrono::system_clock::now();
  alignPV.setSize(400);
  alignVA.setSize(400);
  lastPositionTime = now;
  lastVelocityTime = now;
  lastPosition.setZero();
  lastVelocity.setZero();
}

void
CascadeControllerNode::pid_param_callback(
  cascade_controller::paramsConfig& config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %lf %lf %lf %lf", config.K_pxy, config.K_pz,
           config.K_vxy, config.K_vz);
  ros::NodeHandle nh(getPrivateNodeHandle());

  Eigen::Matrix3d posCoeff;
  Eigen::Matrix3d velCoeff;
  Eigen::Matrix3d accCoeff;

  Eigen::Matrix3d value_default;

  value_default.setZero();

  posCoeff = getMatrixParam(nh, value_default, "position_coeff");
  velCoeff = getMatrixParam(nh, value_default, "velocity_coeff");
  // accCoeff = getMatrixParam(nh, value_default, "acceleration_coeff");

  posCoeff(0, 0) = config.K_pxy;
  posCoeff(1, 0) = config.K_pxy;
  posCoeff(2, 0) = config.K_pz;
  velCoeff(0, 0) = config.K_vxy;
  velCoeff(1, 0) = config.K_vxy;
  velCoeff(2, 0) = config.K_vz;
  std::cout << "after: \n" << posCoeff << std::endl;
  std::cout << velCoeff << std::endl;

  ctrl->getControllerPos()->setCoeff(posCoeff);
  ctrl->getControllerVel()->setCoeff(velCoeff);
  // ctrl->getControllerAcc()->setCoeff(accCoeff);
}

void
CascadeControllerNode::enable_callback(
  const std_msgs::Bool::ConstPtr& enableSwitch)
{
  if (enable != enableSwitch->data)
  {
    enable = enableSwitch->data;
    if (enable)
    {
      //! @todo implement
    }
    else
    {
      //! @todo implement
    }
  }
}

void
CascadeControllerNode::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (resetQuaternion)
  {
    //! @note protection code for all zero odom orentation input
    Eigen::Vector4d test(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    if (test.norm() == 0)
      return;
  }

  Eigen::Vector3d pos;
  pos(0) = odom->pose.pose.position.x;
  pos(1) = odom->pose.pose.position.y;
  pos(2) = odom->pose.pose.position.z;

  current_yaw = tf::getYaw(odom->pose.pose.orientation);

  odomQuaternion = Eigen::Quaterniond(
    odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  odomQuaternion.normalize();

  feedVelocity(0) = odom->twist.twist.linear.x;
  feedVelocity(1) = odom->twist.twist.linear.y;
  feedVelocity(2) = odom->twist.twist.linear.z;

  Eigen::Vector3d palstance;
  palstance(0) = odom->twist.twist.angular.x;
  palstance(1) = odom->twist.twist.angular.y;
  palstance(2) = odom->twist.twist.angular.z;

  Eigen::Quaterniond feedQuaternion = odomQuaternion;

  update_position(pos);
  update_velocity(feedVelocity);
  // usless sofar since palstance controller is not working
  ctrl->setFeedQuaternion(feedQuaternion);
  ctrl->setFeedPalstance(palstance);

  if (resetQuaternion && (!isSimulation))
  {
    if ((fcQuaternion.x() == 0) && (fcQuaternion.y() == 0) &&
        (fcQuaternion.z() == 0) && (fcQuaternion.w() == 0))
    {
      ROS_WARN(
        "fc Quaternion not received yet, cannot reset the orientation align");
    }
    else
    {
      Eigen::Vector3d ea_imu =
        uav_utils::R_to_ypr(fcQuaternion.toRotationMatrix());

      Eigen::Vector3d ea_odom =
        uav_utils::R_to_ypr(odomQuaternion.toRotationMatrix());

      errorQuaternion =
        Eigen::AngleAxisd(ea_imu(0) - ea_odom(0), Eigen::Vector3d::UnitZ());
      resetQuaternion = false;
    }
  }

}

void
CascadeControllerNode::odom_correction_callback(
  const nav_msgs::Odometry::ConstPtr& odom)
{

  Eigen::Quaterniond q(
    odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
    odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  q.normalize();

  pthread_mutex_lock(&correctionMutex);
  correctionPosition(0) = odom->pose.pose.position.x;
  correctionPosition(1) = odom->pose.pose.position.y;
  correctionPosition(2) = odom->pose.pose.position.z;

  correctionVelocity(0) = odom->twist.twist.linear.x;
  correctionVelocity(1) = odom->twist.twist.linear.y;
  correctionVelocity(2) = odom->twist.twist.linear.z;

  correctionPalstance(0) = odom->twist.twist.angular.x;
  correctionPalstance(1) = odom->twist.twist.angular.y;
  correctionPalstance(2) = odom->twist.twist.angular.z;

  correctionQuaternion = q;
  pthread_mutex_unlock(&correctionMutex);
}

void
CascadeControllerNode::flight_status_callback(
  const std_msgs::UInt8::ConstPtr& flightstatus)
{
#ifdef HAVE_DJIOSDK_PRIVATE
  insky = flightstatus->data == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
            ? true
            : false;
#endif
}

void
CascadeControllerNode::imu_acc_world_callback(
  const sensor_msgs::Imu::ConstPtr& imu)
{
  //! @note simulation usage
  Eigen::Quaterniond q(imu->orientation.w, imu->orientation.x,
                       imu->orientation.y, imu->orientation.z);

  Eigen::Vector3d palstance;
  palstance(0) = imu->angular_velocity.x;
  palstance(1) = imu->angular_velocity.y;
  palstance(2) = imu->angular_velocity.z;

  feedAcceleration(0) = imu->linear_acceleration.x;
  feedAcceleration(1) = imu->linear_acceleration.y;
  feedAcceleration(2) = imu->linear_acceleration.z;

  update_acceleration(feedAcceleration);
  //! @todo fuse
  //  ctrl->setFeedPalstance(palstance);
  //  ctrl->setFeedQuaternion(q);
}

void
CascadeControllerNode::imu_callback(const sensor_msgs::Imu::ConstPtr& imu)
{
  Eigen::Quaterniond q(imu->orientation.w, imu->orientation.x,
                       imu->orientation.y, imu->orientation.z);

  Eigen::Vector3d palstance;
  palstance(0) = imu->angular_velocity.x;
  palstance(1) = imu->angular_velocity.y;
  palstance(2) = imu->angular_velocity.z;

  Eigen::Vector3d acc_b(imu->linear_acceleration.x, imu->linear_acceleration.y,
                        imu->linear_acceleration.z);

  fcQuaternion = q;

  feedAcceleration =
    q.toRotationMatrix().transpose() * acc_b - Eigen::Vector3d(0, 0, 1) * 9.81;

  update_acceleration(feedAcceleration);
}

void
CascadeControllerNode::view_callback(const geometry_msgs::Point::ConstPtr& view)
{
  pthread_mutex_lock(&correctionMutex);
  tempViewpoint(0) = view->x;
  tempViewpoint(1) = view->y;
  tempViewpoint(2) = view->z;
  pthread_mutex_unlock(&correctionMutex);
}

void
CascadeControllerNode::thrust_callback(const std_msgs::Float64::ConstPtr& force)
{
  pthread_mutex_lock(&correctionMutex);
  correctionThrust = force->data;
  pthread_mutex_unlock(&correctionMutex);
}

void
CascadeControllerNode::gain_callback(
  const std_msgs::Float64MultiArray::ConstPtr& data)
{
  //! @todo implement
}

void
CascadeControllerNode::thrustObserver(mocka::State state, double djiThrust)
{
  if (insky)
  {
    if (state.velocity(2) == 0)
    {

      double updateGain = 0.004 / djiThrust;

      double add = updateGain * (feedVelocity(2) * mass);

      if (firstObserve)
      {
        dirtyCounter++; // avoid near time input falut
        add *= 1 / djiThrust;
        if ((std::abs(feedAcceleration(2)) < 0.2 &&
             std::abs(feedVelocity(2)) < 0.4 && dirtyCounter > 100))
          firstObserve = false;
      }
      else
      {
        double currerentIntergrade =
          ctrl->getControllerPos()->getZController()->getIntergrade();
        add += 0.0002 * (lastIntergrade - currerentIntergrade);
        lastIntergrade = currerentIntergrade;
        //        std::cout << "iz: " << currerentIntergrade << std::endl;
      }

      maxThrust += add;
      if (maxThrust < 0)
        maxThrust -= add;
      //      std::cout << "update thrust " << add << " " << djiThrust <<
      //      std::endl;
    }
  }
}

void
CascadeControllerNode::position_command_callback(
  const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{

  ros::Time now = ros::Time::now();

  mocka::State state;
  state.position(0) = cmd->position.x;
  state.position(1) = cmd->position.y;
  state.position(2) = cmd->position.z;

  state.velocity(0) = cmd->velocity.x;
  state.velocity(1) = cmd->velocity.y;
  state.velocity(2) = cmd->velocity.z;

  state.acceleration(0) = cmd->acceleration.x;
  state.acceleration(1) = cmd->acceleration.y;
  state.acceleration(2) = cmd->acceleration.z;

  //! @todo ugly, need better yaw impl
  {
    double curC   = std::acos(tempViewpoint(0));
    double curS   = std::asin(tempViewpoint(1));
    double curYaw = curC;
    if (curS < 0)
      curYaw         = -curYaw;
    tempViewpoint(0) = std::cos(curYaw + cmd->yaw_dot * 0.02);
    tempViewpoint(1) = std::sin(curYaw + cmd->yaw_dot * 0.02);
  }

  pthread_mutex_lock(&correctionMutex);
  state.viewPoint  = tempViewpoint;
  state.quaternion = correctionQuaternion;
  state.palstance  = correctionPalstance;
  state.thrust     = correctionThrust;
  pthread_mutex_unlock(&correctionMutex);

  ctrl->setInput(state);
  ctrl->spinOnce();
  mocka::State output;
  output = ctrl->getOutput();

  if (enable)
  {
    bool feedResult = true;
    feedResult &= feed_position();
    feedResult &= feed_velocity();
    feedResult &= feed_acceleration();
    if (!feedResult)
      return;

    quadrotor_msgs::SO3Command so3_command; //! @note for simulator

    so3_command.header.stamp    = ros::Time::now();
    so3_command.header.frame_id = frame_id;

    so3_command.force.x = output.acceleration(0) * mass;
    so3_command.force.y = output.acceleration(1) * mass;
    so3_command.force.z = output.acceleration(2) * mass + mocka::SO3::g * mass;

    so3_command.orientation.x = output.quaternion.x();
    so3_command.orientation.y = output.quaternion.y();
    so3_command.orientation.z = output.quaternion.z();
    so3_command.orientation.w = output.quaternion.w();

    for (int i = 0; i < 3; i++)
    {
      so3_command.kR[i]  = kR[i];
      so3_command.kOm[i] = kOm[i];
    }
    so3_command.aux.current_yaw          = current_yaw;
    so3_command.aux.kf_correction        = corrections[0];
    so3_command.aux.angle_corrections[0] = corrections[1];
    so3_command.aux.angle_corrections[1] = corrections[2];
    so3_command.aux.enable_motors        = true;
    so3_command.aux.use_external_yaw     = false;

    so3_command_pub.publish(so3_command);

    sensor_msgs::Joy dji_command_so3; //! @note for dji ros wrapper

    dji_command_so3.header.stamp    = now;
    dji_command_so3.header.frame_id = std::string("FRD");

    uint8_t flag;

#ifdef HAVE_DJIOSDK_PRIVATE
    flag = DJI::OSDK::Control::VERTICAL_THRUST |
           DJI::OSDK::Control::HORIZONTAL_ANGLE | //
           DJI::OSDK::Control::YAW_ANGLE |        //
           DJI::OSDK::Control::HORIZONTAL_BODY |
           DJI::OSDK::Control::STABLE_DISABLE;
#endif

    Eigen::Vector3d rotation = uav_utils::R_to_ypr(
      errorQuaternion * output.quaternion.toRotationMatrix());

    //! @note update thrust
    double djiThrust = force2Percentage(output.thrust * mass);

    thrustObserver(state, djiThrust);

    dji_command_so3.axes.push_back(rotation(2));       // x
    dji_command_so3.axes.push_back(rotation(1));       // y
    dji_command_so3.axes.push_back(djiThrust * 100.0); // z
    dji_command_so3.axes.push_back(rotation(0));       // w
    dji_command_so3.axes.push_back(flag);

    attitude_command_pub.publish(dji_command_so3);

    sensor_msgs::Joy dji_command_palstance;
    dji_command_palstance.header.stamp    = now;
    dji_command_palstance.header.frame_id = std::string("FRD");

#ifdef HAVE_DJIOSDK_PRIVATE
    flag = DJI::OSDK::Control::VERTICAL_THRUST |
           DJI::OSDK::Control::HORIZONTAL_ANGULAR_RATE | //
           DJI::OSDK::Control::YAW_RATE |                //
           DJI::OSDK::Control::HORIZONTAL_BODY |
           DJI::OSDK::Control::STABLE_DISABLE;
#endif

    dji_command_palstance.axes.push_back(output.palstance(0)); // x
    dji_command_palstance.axes.push_back(output.palstance(1)); // y
    dji_command_palstance.axes.push_back(djiThrust * 100.0);   // z
    dji_command_palstance.axes.push_back(output.palstance(2)); // w
    dji_command_palstance.axes.push_back(flag);

    palstance_command_pub.publish(dji_command_palstance);

    //! @note debug info
    geometry_msgs::Point e;
    e.x = ctrl->getControllerPos()->getError();
    e.y = ctrl->getControllerVel()->getError();
    e.z = ctrl->getControllerAcc()->getError();
    translation_error_pub.publish(e);
  }
  else
  {
    // manual control
    sensor_msgs::Joy dji_command_manual;
    dji_command_manual.header.stamp    = now;
    dji_command_manual.header.frame_id = std::string("FRD");

    uint8_t flag;

#ifdef HAVE_DJIOSDK_PRIVATE
    flag = DJI::OSDK::Control::VERTICAL_VELOCITY |
           DJI::OSDK::Control::HORIZONTAL_ANGLE | //
           DJI::OSDK::Control::YAW_RATE |         //
           DJI::OSDK::Control::HORIZONTAL_BODY |
           DJI::OSDK::Control::STABLE_DISABLE;
#endif

    dji_command_manual.axes.push_back(cmd->position.x); // x
    dji_command_manual.axes.push_back(cmd->position.y); // y
    dji_command_manual.axes.push_back(cmd->position.z); // z
    dji_command_manual.axes.push_back(cmd->yaw_dot);    // w
    dji_command_manual.axes.push_back(flag);
    manual_command_pub.publish(dji_command_manual);
  }
}

/*! @note for adapt dji osdk ros
 * */

double
CascadeControllerNode::force2Percentage(const double force) const
{
  double ans = force / maxThrust;
  if (ans < 0.0)
    return 0.0;
  if (ans > 1.0)
    return 1.0;
  return ans;
}

double
CascadeControllerNode::getMass() const
{
  return mass;
}

void
CascadeControllerNode::setMass(double value)
{
  mass = value;
}
