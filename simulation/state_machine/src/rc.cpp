#include "rc.hpp"

#include <math.h>

#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <Eigen/Eigen>

using namespace mocka;

RC::RC()
  : statemachine(nullptr)
#ifdef HAVE_DJIOSDK_PRIVATE
  , authorityClient("dji_sdk/action/authority", true)
  , motorSwitchClient("dji_sdk/action/motor_switch", true)
  , flightTaskClient("dji_sdk/action/flight_task", true)
#endif
  , id(0)
  , currentControlling(0)
  , simulation(false)
  , takeoffHover(false)
{
}

RC::RC(ros::NodeHandle& nh, OperationalLogic::StateMachine* sm)
  : logic(sm)
#ifdef HAVE_DJIOSDK_PRIVATE
  , authorityClient("dji_sdk/action/authority", true)
  , motorSwitchClient("dji_sdk/action/motor_switch", true)
  , flightTaskClient("dji_sdk/action/flight_task", true)
#endif
  , id(0)
  , currentControlling(0)
  , simulation(false)
  , takeoffHover(false)
{
  // ros related

  int mission_init;

  //! @todo take off mission and landing mission
  nh.param("simulation", simulation, false);
  nh.param("mission/button_A", missionA, std::string("missionA"));
  nh.param("mission/button_B", missionB, std::string("missionB"));
  nh.param("mission/button_X", missionX, std::string("missionX"));
  nh.param("mission/button_Y", missionY, std::string("missionY"));
  nh.param("mission/init_mission", mission_init, 0);
  nh.param("z_hover", zOffset, 0.8);
  nh.param("yaw", hasYawControl, false);
  nh.param("horizontal_gain", horizontalGain, 1.2);
  nh.param("vertical_gain", verticalGain, 1.2);
  nh.param("yaw_gain", yawGain, 0.8);

  nh.param("joymap_thrust", joyMap[JOY_THRUST], 1);
  nh.param("joymap_yaw", joyMap[JOY_YAW], 0);
  nh.param("joymap_pitch", joyMap[JOY_PITCH], 4);
  nh.param("joymap_roll", joyMap[JOY_ROLL], 3);

  nh.param("joymap_a", joyMap[JOY_A], 0);
  nh.param("joymap_b", joyMap[JOY_B], 1);
  nh.param("joymap_x", joyMap[JOY_X], 2);
  nh.param("joymap_y", joyMap[JOY_Y], 3);

  nh.param("joymap_rb", joyMap[JOY_RB], 5);
  nh.param("joymap_lb", joyMap[JOY_LB], 4);

  nh.param("joymap_start", joyMap[JOY_START], 7);
  nh.param("joymap_back", joyMap[JOY_BACK], 6);

#ifdef HAVE_DJIOSDK_PRIVATE
  sub_dji = nh.subscribe("dji_sdk/rc", 10, &RC::fromOSDK, this);
  sub_ctrl_dev =
    nh.subscribe("dji_sdk/control_device", 10, &RC::sdkControlDevice, this);
#endif

  sub_joy  = nh.subscribe("rc_input", 10, &RC::fromJoy, this);
  sub_odom = nh.subscribe("odom", 10, &RC::odomCallback, this);

  ROS_WARN("Waiting for action server to start.");
  if (simulation == false)
  {
#ifdef HAVE_DJIOSDK_PRIVATE
    authorityClient.waitForServer();
    motorSwitchClient.waitForServer();
    flightTaskClient.waitForServer();
#endif
    ROS_WARN("Action server started, sending goal.");
  }

  mission_publisher = nh.advertise<std_msgs::String>("rc/mission", 10, true);
  position_cmd_publisher =
    nh.advertise<quadrotor_msgs::PositionCommand>("rc/position", 20, true);
  attitude_cmd_publisher =
    nh.advertise<quadrotor_msgs::SO3Command>("rc/attitude", 20, true);
  //! @todo takeoff/landing related logic
  enable_control_publisher =
    nh.advertise<std_msgs::Bool>("rc/enable_control", 5, true);
  // state machine related
  statemachine = new StateMachine{ logger };

  statemachine->getInfo().setHostRC(this);

  joy.sticks.reserve(STICK_NUMBER);
  joy.sticks.resize(STICK_NUMBER);
  joy.buttons.reserve(BUTTON_NUMBER);
  joy.buttons.resize(BUTTON_NUMBER);

  osdk.sticks.reserve(STICK_NUMBER);
  osdk.sticks.resize(STICK_NUMBER);
  osdk.buttons.reserve(BUTTON_NUMBER);
  osdk.buttons.resize(BUTTON_NUMBER);

  for (auto& it : joy.sticks)
    it = 0;
  for (auto& it : osdk.sticks)
    it = 0;
  for (auto& it : joy.buttons)
    it = 0;
  for (auto& it : osdk.buttons)
    it = 0;

  osdk.buttons[BUTTON_ISCONNECTED] = 2;

  joy.buttons[BUTTON_MISSION] = mission_init;
}

RC::~RC()
{
  if (statemachine != nullptr)
    delete statemachine;
}

void
RC::publish_mission(int mission)
{
  if (mission != 0)
  {
    std_msgs::String mission_string;

    std::string mission_ans;
    switch (mission)
    {
      case -1:
        mission_ans = "stop";
        break;
      case 1:
        mission_ans = missionA;
        setMissionCurrent(missionA);
        break;
      case 2:
        mission_ans = missionB;
        setMissionCurrent(missionB);
        break;
      case 4:
        mission_ans = missionX;
        setMissionCurrent(missionX);
        break;
      case 8:
        mission_ans = missionY;
        setMissionCurrent(missionY);
        break;
      default:
        break;
    }

    mission_string.data = mission_ans;
    mission_publisher.publish(mission_string);
  }

  //! @note desabled because joy input may needed by missions
  Eigen::Vector4d stick_vector(joy.sticks[STICK_THRUST], joy.sticks[STICK_YAW],
                               joy.sticks[STICK_PITCH], joy.sticks[STICK_ROLL]);

  if (stick_vector.norm() != 0)
  {
    // this will out put very much times, for safety reason
    std_msgs::String mission_string;

    std::string mission_ans = "interrupt";
    eventQueue.push(RC::EVENT_REQUEST_EXIT_MISSION);

    mission_string.data = mission_ans;
    mission_publisher.publish(mission_string);
  }
}

void
RC::fromJoy(const sensor_msgs::Joy::ConstPtr& data)
{
  //! @note Back betop RT LT and stick button pannel are not used;

  // todo configurational
  joy.sticks[STICK_THRUST] = data->axes.at(joyMap[JOY_THRUST]);
  joy.sticks[STICK_YAW]    = data->axes.at(joyMap[JOY_YAW]);
  joy.sticks[STICK_PITCH]  = data->axes.at(joyMap[JOY_PITCH]);
  joy.sticks[STICK_ROLL]   = data->axes.at(joyMap[JOY_ROLL]);

  int mission = 0;
  mission |= data->buttons.at(joyMap[JOY_A]);      // A
  mission |= data->buttons.at(joyMap[JOY_B]) << 1; // B
  mission |= data->buttons.at(joyMap[JOY_X]) << 2; // X
  mission |= data->buttons.at(joyMap[JOY_Y]) << 3; // Y

  joy.buttons[BUTTON_MODE]        = 1; // joy stick must use OSDK mode
  joy.buttons[BUTTON_SWITCH_UP]   = data->buttons.at(joyMap[JOY_RB]); // RB
  joy.buttons[BUTTON_SWITCH_DOWN] = data->buttons.at(joyMap[JOY_LB]); // LB

  int start = joy.buttons[BUTTON_START];
  if (data->buttons.at(joyMap[JOY_START]) == 1) // start
  {
    eventQueue.push(RC::EVENT_REQUEST_ENTER_MISSION);
    start   = 1;
    mission = joy.buttons[BUTTON_MISSION];
    publish_mission(-1);
    publish_mission(mission);
  }
  if (data->buttons.at(joyMap[JOY_BACK])) // back
  {
    eventQueue.push(RC::EVENT_REQUEST_EXIT_MISSION);
    start   = 0;
    mission = joy.buttons[BUTTON_MISSION];
    publish_mission(-1);
  }

  joy.buttons[BUTTON_START]       = start;
  joy.buttons[BUTTON_ISCONNECTED] = 1;
  if (mission != 0)
    joy.buttons[BUTTON_MISSION] = mission;

  //! @todo parameterize
  if (data->buttons.at(9) == 1 && data->buttons.at(10) == 0)
  {
    statemachine->process_event(Logic::MotorOnEvent());
  }
  if (data->buttons.at(9) == 0 && data->buttons.at(10) == 1)
  {
    statemachine->process_event(Logic::MotorOffEvent());
  }
  if (data->buttons.at(9) == 1 && data->buttons.at(10) == 1)
    eventQueue.push(RC::EVENT_REQUEST_RETURN);

  if (lastSwitchLeft == true && data->buttons.at(joyMap[JOY_LB]) == 0)
  {
    currentControlling--;
  }
  if (lastSwitchRight == true && data->buttons.at(joyMap[JOY_RB]) == 0)
  {
    currentControlling++;
  }
  lastSwitchLeft  = data->buttons.at(joyMap[JOY_LB]);
  lastSwitchRight = data->buttons.at(joyMap[JOY_RB]);
}

void
RC::odomCallback(const nav_msgs::Odometry::ConstPtr& data)
{
  odom = *data;
}

#ifdef HAVE_DJIOSDK_PRIVATE
void
RC::fromOSDK(const sensor_msgs::Joy::ConstPtr& data)
{
  //  std::cout << "From OSDK" << std::endl;
  osdk.sticks[STICK_THRUST] = data->axes.at(3);
  osdk.sticks[STICK_YAW]    = data->axes.at(2);
  osdk.sticks[STICK_PITCH]  = data->axes.at(1);
  osdk.sticks[STICK_ROLL]   = data->axes.at(0);

  // DJI RC does not have enough channel to input different mission
  int mission = 0;

  osdk.buttons[BUTTON_MODE]        = (data->axes.at(4) == 10000) ? 1 : 0;
  osdk.buttons[BUTTON_SWITCH_UP]   = 0;
  osdk.buttons[BUTTON_SWITCH_DOWN] = 0;
  osdk.buttons[BUTTON_START]   = (data->axes.at(5) == -5000) ? 0 : 1; // START
  osdk.buttons[BUTTON_MISSION] = mission;
  osdk.buttons[BUTTON_ISCONNECTED] = (data->axes.at(5) == 0) ? 0 : 1;
}

void
RC::Logic::obtainControlCallback(const actionlib::SimpleClientGoalState& state,
                                 const dji_sdk::AuthorityResultConstPtr& result)
{
  ROS_INFO("Answer: %d", result->accept);
  enter = result->accept;
}

void
RC::Logic::releaseControlCallback(
  const actionlib::SimpleClientGoalState& state,
  const dji_sdk::AuthorityResultConstPtr& result)
{
  ROS_INFO("Answer: %d", result->accept);
  exit = result->accept;
}

void
RC::Logic::motorOnCallback(const actionlib::SimpleClientGoalState&   state,
                           const dji_sdk::MotorSwitchResultConstPtr& result)
{
  if (result->accept)
  {
    hostRC->x0 = hostRC->odom.pose.pose.position.x;
    hostRC->y0 = hostRC->odom.pose.pose.position.y;
    hostRC->z0 = hostRC->odom.pose.pose.position.z;
    ROS_INFO("TAKE OFF %lf %lf %lf", hostRC->x0, hostRC->y0, hostRC->z0);
    hostRC->takeoffHover = true;
  }
}

void
RC::Logic::motorOffCallback(const actionlib::SimpleClientGoalState&   state,
                            const dji_sdk::MotorSwitchResultConstPtr& result)
{
}

void
RC::Logic::landingCallback(const actionlib::SimpleClientGoalState&  state,
                           const dji_sdk::FlightTaskResultConstPtr& result)
{
}
void
RC::authorityDone(const actionlib::SimpleClientGoalState& state,
                  const dji_sdk::AuthorityResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %d", result->accept);
}

void
RC::sdkControlDevice(const dji_sdk::ControlDevice::ConstPtr& data)
{
  sdkControlling = (data->controlDevice == 2 ? true : false);
  if (sdkControlling)
  {
    //! @note done by state machine
    //    eventQueue.push(EVENT_EXITSDK);
    statemachine->process_event(Logic::AlreadyEnterSDKEvent());
  }
  else
  {
    //! @note done by state machine
    //    eventQueue.push(EVENT_ENTERSDK);
    statemachine->process_event(Logic::AlreadyEnterRCEvent());
  }
}
#endif

RC::Data
RC::getInput() const
{
  return osdk;
}

RC::EVENT
RC::process()
{
  statemachine->process_event(Logic::PollEvent());
  if (eventQueue.empty())
    return EVENT_NONE;
  auto ans = eventQueue.front();
  eventQueue.pop();
  return ans;
}

std::string
RC::getMissionCurrent() const
{
  return missionCurrent;
}

void
RC::setMissionCurrent(const std::string& value)
{
  missionCurrent = value;
}

int
RC::getId() const
{
  return id;
}

void
RC::setID(int value)
{
  id = value;
}

void
RC::Logic::publishCommands()
{
  quadrotor_msgs::PositionCommand cmd_pos;

  cmd_pos.header.stamp = ros::Time::now();

  //! @note ugly statemachine in side SML
  if (hostRC->takeoffHover)
  {
    cmd_pos.position.x = hostRC->x0;
    cmd_pos.position.y = hostRC->y0;
    cmd_pos.position.z = hostRC->z0 + hostRC->zOffset;
    if (cmd_pos.position.z - hostRC->odom.pose.pose.position.z > 0.05)
    {
      cmd_pos.position.z = hostRC->odom.pose.pose.position.z + 0.1;
      //! @note debug
      //      ROS_WARN("TAKEING OFF %lf %lf", cmd_pos.position.z - hostRC->z0,
      //               hostRC->zOffset);
    }
  }
  else
  {
    cmd_pos.position.x = std::numeric_limits<double>::quiet_NaN();
    cmd_pos.position.y = std::numeric_limits<double>::quiet_NaN();
    cmd_pos.position.z = std::numeric_limits<double>::quiet_NaN();
  }
  if (hostRC->currentControlling == 0 ||
      hostRC->currentControlling == hostRC->id)
  {
    //! @todo parameterized
    Eigen::Vector3d vel;

    vel(0) = hostRC->joy.sticks[STICK_PITCH] * hostRC->horizontalGain; // - 0.5;
    vel(1) = hostRC->joy.sticks[STICK_ROLL] * hostRC->horizontalGain;  // - 0.5;
    vel(2) = hostRC->joy.sticks[STICK_THRUST] * hostRC->verticalGain;  // - 0.5;
    if (hostRC->hasYawControl)
    {
      cmd_pos.yaw_dot = hostRC->joy.sticks[STICK_YAW] * hostRC->yawGain;
      Eigen::Quaterniond q;
      q.x() = hostRC->odom.pose.pose.orientation.x;
      q.y() = hostRC->odom.pose.pose.orientation.y;
      q.z() = hostRC->odom.pose.pose.orientation.z;
      q.w() = hostRC->odom.pose.pose.orientation.w;
      vel   = q.toRotationMatrix() * vel;
    }
    else
    {
      cmd_pos.yaw_dot = 0;
    }
    cmd_pos.velocity.x = vel(0);
    cmd_pos.velocity.y = vel(1);
    cmd_pos.velocity.z = vel(2);
    if ((cmd_pos.velocity.x != 0) || (cmd_pos.velocity.y != 0))
    {
      if (hostRC->takeoffHover)
        ROS_WARN("EXIT Takeoff Hover");
      hostRC->takeoffHover = false;
    }
  }
  else
  {
    cmd_pos.velocity.x = 0; // - 0.5;
    cmd_pos.velocity.y = 0; // - 0.5;
    cmd_pos.velocity.z = 0; // - 0.5;
  }
  cmd_pos.acceleration.x = 0;
  cmd_pos.acceleration.y = 0;
  cmd_pos.acceleration.z = 0;

  cmd_pos.yaw = 0;

  cmd_pos.kx[0] = 1;
  cmd_pos.kx[1] = 1;
  cmd_pos.kx[2] = 1;

  cmd_pos.kv[0] = 1;
  cmd_pos.kv[1] = 1;
  cmd_pos.kv[2] = 1;

  //  cmd_pos.yaw
  hostRC->position_cmd_publisher.publish(cmd_pos);
  quadrotor_msgs::SO3Command cmd_attitude;
  //! @todo implement for none-VINS situation
  hostRC->attitude_cmd_publisher.publish(cmd_attitude);
}

void
RC::Logic::sdkPoll()
{
  static int counter = 0;
  //! @todo switch for joy; however I forget what's this todo for @ 2019/04/09
  //!   static int counter = 0;
  // clang-format off
  using namespace boost::sml;
  if (hostRC->logic->is("mission"_s))
  {
    hostRC->takeoffHover = false;
    if (counter++ % 100 == 0)
      ROS_INFO("SDK Mission");
    return; //! @note returned
  }
  else
  {   
   if (counter++ % 100 == 0)
    ROS_INFO("SDK");
  }
  // clang-format on
  publishCommands();
}

void
RC::Logic::rcPoll()
{
  static int counter = 0;
  if (counter++ % 100 == 0)
    std::cout << "RC" << std::endl;
  //  ROS_INFO("RC");

  //! @todo
  //  publish RC command;
}

bool
RC::Logic::requestEnterSDK()
{
  if (hostRC->simulation)
  {
    return true;
  }
  else
  {
    if (!haveLB2())
      return true;
    else
    {
      if (hostRC->osdk.buttons[BUTTON_MODE] == 1)
        return true;
      return false;
    }
  }
}

bool
RC::Logic::isSDK()
{
  return hostRC->sdkControlling;
}

bool
RC::Logic::requestEnterRC()
{
  if (!haveLB2())
    return false;
  auto total = std::abs(hostRC->osdk.sticks[STICK_PITCH]) +
               std::abs(hostRC->osdk.sticks[STICK_ROLL]) +
               std::abs(hostRC->osdk.sticks[STICK_THRUST]) +
               std::abs(hostRC->osdk.sticks[STICK_YAW]);
  //  ROS_INFO("Total: %lf", total);
  if (total > 0.05)
    return true;
  return false;
}

bool
RC::Logic::haveLB2()
{
  return hostRC->osdk.buttons[BUTTON_ISCONNECTED] ? true : false;
}

void
RC::Logic::obtainControl()
{
  if (hostRC->simulation)
    return;
#ifdef HAVE_DJIOSDK_PRIVATE
  dji_sdk::AuthorityGoal goal;
  goal.reqSDKControl = 1;
  hostRC->authorityClient.sendGoal(
    goal, boost::bind(&RC::Logic::obtainControlCallback, this, _1, _2),
    AuthorityClient::SimpleActiveCallback(),
    AuthorityClient::SimpleFeedbackCallback());
#endif
  std::cout << "Enter Request enter SDK" << std::endl;
  enter = false;
}

void
RC::Logic::releaseControl()
{
  if (hostRC->simulation)
    return;
#ifdef HAVE_DJIOSDK_PRIVATE
  dji_sdk::AuthorityGoal goal;
  goal.reqSDKControl = 0;
  hostRC->authorityClient.sendGoal(
    goal, boost::bind(&RC::Logic::releaseControlCallback, this, _1, _2),
    AuthorityClient::SimpleActiveCallback(),
    AuthorityClient::SimpleFeedbackCallback());
#endif
  std::cout << "Enter Request exit SDK" << std::endl;
  exit = false;
}

void
RC::Logic::setHostRC(RC* value)
{
  hostRC = value;
}
