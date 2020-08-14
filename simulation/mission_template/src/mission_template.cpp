#include "mission_template.hpp"

#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <uav_utils/geometry_utils.h>

#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(MissionTemplateNode, nodelet::Nodelet);


MissionTemplateNode::MissionTemplateNode()
  : statemachine(new StateMachine{ logger })
  , name("MissionTemplateNode")
{
  using namespace boost::sml;

  Logic* l = &statemachine->getInfo();
  l->setHost(this);
}

MissionTemplateNode::~MissionTemplateNode()
{
  delete statemachine;
}

void
MissionTemplateNode::onInit()
{
  lastPosition.setZero();
  lastVelocity.setZero();
  lastQuaternion = Eigen::Quaterniond(1, 0, 0, 0);

  ros::NodeHandle nh(getPrivateNodeHandle());

  std::string quadrotor_name;
  nh.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  // publishers
  position_command_pub =
    nh.advertise<quadrotor_msgs::PositionCommand>("position_command", 2);
  path_pub = nh.advertise<nav_msgs::Path>("path", 2);

  // subscribers
  mission_poller_sub =
    nh.subscribe("poller", 10, &MissionTemplateNode::poller_callback, this,
                 ros::TransportHints().tcpNoDelay());
  mission_enable_sub =
    nh.subscribe("enable", 10, &MissionTemplateNode::enable_callback, this,
                 ros::TransportHints().tcpNoDelay());
  joy_sub = nh.subscribe("joy", 10, &MissionTemplateNode::joy_callback, this,
                         ros::TransportHints().tcpNoDelay());
  odom_sub = nh.subscribe("odom", 10, &MissionTemplateNode::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());

  resetTime();
  resetPath();
}

void
MissionTemplateNode::startMission()
{
  statemachine->process_event(Logic::StartEvent());
}

void
MissionTemplateNode::stopMission()
{
  statemachine->process_event(Logic::StopEvent());
}

void
MissionTemplateNode::resetTime()
{
  //  ROS_INFO("RESET TIME");
  startTime = std::chrono::system_clock::now();
}

void
MissionTemplateNode::resetPath()
{
  path.header.stamp    = ros::Time::now();
  path.header.frame_id = "map";

  path.poses.clear();
}

void
MissionTemplateNode::run(double dt)
{
  ROS_INFO("%s is running", name.c_str());
}

void
MissionTemplateNode::clean()
{
  ROS_INFO("%s is cleaning", name.c_str());
  path.poses.clear();
}

bool
MissionTemplateNode::start()
{
  resetPath();
  resetTime();
  return true;
}

void
MissionTemplateNode::poll()
{
}

void
MissionTemplateNode::joy(const sensor_msgs::Joy::ConstPtr& data)
{
}

std::string
MissionTemplateNode::getName() const
{
  return name;
}

void
MissionTemplateNode::setName(const std::string& value)
{
  name = value;
}

void
MissionTemplateNode::poller_callback(const std_msgs::String::ConstPtr& data)
{
  using namespace boost::sml;

  poll();
  if (data->data == "run")
    statemachine->process_event(Logic::PollEvent());
}

void
MissionTemplateNode::enable_callback(const std_msgs::String::ConstPtr& data)
{
  if (data->data == name)
  {
    startMission();
    return;
  }
  if (data->data == "stop")
  {
    stopMission();
    return;
  }
}

void
MissionTemplateNode::joy_callback(const sensor_msgs::Joy::ConstPtr& data)
{
  //! @todo implement
  // clang-format off
  using namespace boost::sml;
  if(statemachine->is("running"_s))
  // clang-format on
  {
    joy(data);
  }
  lastJoy = *data;
}

void
MissionTemplateNode::odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
  Eigen::Vector3d position_0;
  position_0(0) = data->pose.pose.position.x;
  position_0(1) = data->pose.pose.position.y;
  position_0(2) = data->pose.pose.position.z;

  Eigen::Quaterniond quaternion_0(
    data->pose.pose.orientation.w, data->pose.pose.orientation.x,
    data->pose.pose.orientation.y, data->pose.pose.orientation.z);

  Eigen::Vector3d velocity_0;
  velocity_0(0) = data->twist.twist.linear.x;
  velocity_0(1) = data->twist.twist.linear.y;
  velocity_0(2) = data->twist.twist.linear.z;

  if (position.size() == maxStorage)
    position.pop();
  if (velocity.size() == maxStorage)
    velocity.pop();
  if (quaternion.size() == maxStorage)
    quaternion.pop();

  position.push(position_0);
  velocity.push(velocity_0);
  quaternion.push(quaternion_0);
}

std::queue<Eigen::Quaterniond>
MissionTemplateNode::getQuaternion() const
{
  return quaternion;
}

Eigen::Vector3d
MissionTemplateNode::readAveragePosition(int maxRange)
{
  Eigen::Vector3d ans;

  ans.setZero();

  int counter = 0;
  while (position.size() > maxRange + 1)
  {
    position.pop();
  }
  while (!position.empty())
  {
    ans += position.front();
    position.pop();
    counter++;
  }

  if (counter)
  {
    ans /= static_cast<double>(counter);
    lastPosition = ans;
    return ans;
  }
  else
  {
    return lastPosition;
  }
}

Eigen::Vector3d
MissionTemplateNode::readAverageVelocity(int maxRange)
{
  Eigen::Vector3d ans;

  ans.setZero();

  int counter = 0;
  while (velocity.size() > maxRange + 1)
  {
    velocity.pop();
  }
  while (!velocity.empty())
  {
    ans += velocity.front();
    velocity.pop();
    counter++;
  }

  if (counter)
  {
    ans /= static_cast<double>(counter);
    lastVelocity = ans;
    return ans;
  }
  else
  {
    return lastVelocity;
  }
}

Eigen::Quaterniond
MissionTemplateNode::readAverageQuaternion(int maxRange)
{
  Eigen::Quaterniond ans(1, 0, 0, 0);

  Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

  int counter = 0;
  while (quaternion.size() > maxRange + 1)
  {
    quaternion.pop();
  }
  while (!quaternion.empty())
  {

    Eigen::Quaterniond q_tmp = quaternion.front();
    Eigen::Vector4d    v_tmp = q_tmp.coeffs();

    A += v_tmp * v_tmp.transpose();

    quaternion.pop();
    counter++;
  }

  if (counter)
  {
    A /= static_cast<double>(counter);
    Eigen::EigenSolver<Eigen::Matrix4d> es(A);
    Eigen::Matrix<std::complex<double>, 4, 1> mat(es.eigenvalues());
    int index;
    mat.real().maxCoeff(&index);
    Eigen::Vector4d largestEigenVector(
      es.eigenvectors().real().block(0, index, 4, 1));
    ans.coeffs()   = largestEigenVector;
    lastQuaternion = ans;
    return ans;
  }
  else
  {
    return lastQuaternion;
  }
}

void
MissionTemplateNode::publishCommand(const Eigen::Vector3d& position_des,
                                    const Eigen::Vector3d& velocity_des,
                                    const Eigen::Vector3d& acceleration_des)
{
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();

  cmd.position.x = position_des(0);
  cmd.position.y = position_des(1);
  cmd.position.z = position_des(2);

  cmd.velocity.x = velocity_des(0);
  cmd.velocity.y = velocity_des(1);
  cmd.velocity.z = velocity_des(2);

  cmd.acceleration.x = acceleration_des(0);
  cmd.acceleration.y = acceleration_des(1);
  cmd.acceleration.z = acceleration_des(2);

  cmd.yaw     = 0;
  cmd.yaw_dot = 0;

  cmd.kx[0] = 1;
  cmd.kx[1] = 1;
  cmd.kx[2] = 1;

  cmd.kv[0] = 1;
  cmd.kv[1] = 1;
  cmd.kv[2] = 1;

  position_command_pub.publish(cmd);

  geometry_msgs::PoseStamped p;
  p.header.stamp    = ros::Time::now();
  p.pose.position.x = position_des(0);
  p.pose.position.y = position_des(1);
  p.pose.position.z = position_des(2);

  path.poses.push_back(p);
  path_pub.publish(path);
}

uint32_t
MissionTemplateNode::getPriority() const
{
  return priority;
}

void
MissionTemplateNode::setPriority(const uint32_t& value)
{
  priority = value;
}

std::queue<Eigen::Vector3d>
MissionTemplateNode::getVelocity() const
{
  return velocity;
}

std::queue<Eigen::Vector3d>
MissionTemplateNode::getPosition() const
{
  return position;
}

MissionTemplateNode*
MissionTemplateNode::Logic::getHost() const
{
  return host;
}

void
MissionTemplateNode::Logic::setHost(MissionTemplateNode* value)
{
  host = value;
  host->statemachine->process_event(InitEvent());
}
