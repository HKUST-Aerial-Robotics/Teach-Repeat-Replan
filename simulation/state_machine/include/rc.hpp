#ifndef RC_HPP
#define RC_HPP

#ifdef HAVE_DJIOSDK
#define HAVE_DJIOSDK_PRIVATE
#include <dji_sdk/AuthorityAction.h>
#include <dji_sdk/ControlDevice.h>
#include <dji_sdk/FlightTaskAction.h>
#include <dji_sdk/MotorSwitchAction.h>
#endif

#include <actionlib/client/simple_action_client.h>
#include <array>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <statemachine.hpp>
#include <vector>

namespace mocka
{

//! @todo decoupling with ros by add a ros wrapper
class RC
{
public:
  typedef struct Data
  {
    std::vector<double> sticks;
    std::vector<int>    buttons;
    // todo  time
  } Data;

public:
  typedef enum JOYMAP {
    JOY_THRUST = 0,
    JOY_YAW    = 1,
    JOY_PITCH  = 2,
    JOY_ROLL   = 3,
    JOY_A      = 4,
    JOY_B      = 5,
    JOY_X      = 6,
    JOY_Y      = 7,
    JOY_RB     = 8,
    JOY_LB     = 9,
    JOY_START  = 10,
    JOY_BACK   = 11,
    JOY_ALL    = 12 // please manualy growth this number
  } JOYMAP;

  typedef enum STICK {
    STICK_YAW    = 0,
    STICK_THRUST = 1,
    STICK_ROLL   = 2,
    STICK_PITCH  = 3,
    STICK_NUMBER = 4 // please manualy growth this number
  } STICK_NAME;

  typedef enum BUTTON { //
    BUTTON_MODE        = 0,
    BUTTON_SWITCH_UP   = 1,
    BUTTON_SWITCH_DOWN = 2,
    BUTTON_START       = 3,
    BUTTON_MISSION     = 4,
    BUTTON_ISCONNECTED = 5,
    BUTTON_NUMBER      = 6 // please manualy growth this number
  } BUTTON_NAME;

  typedef enum EVENT { //
    EVENT_NONE,
    EVENT_ENTERSDK,
    EVENT_EXITSDK,
    EVENT_REQUEST_TAKEOFF,
    EVENT_REQUEST_LANDING,
    EVENT_REQUEST_ENTER_MISSION,
    EVENT_REQUEST_EXIT_MISSION,
    EVENT_REQUEST_RETURN
  } EVENT_NAME;

public:
  class Logic
  {

  public:
    // clang-format off
    struct RuntimeEvent         {int id = 0;};
    struct PollEvent            {static constexpr auto id = 0; };
    struct RequestEnterSDKEvent {static constexpr auto id = 1; };
    struct RequestEnterRCEvent  {static constexpr auto id = 2; };
    struct AlreadyEnterSDKEvent {static constexpr auto id = 3; };
    struct AlreadyEnterRCEvent  {static constexpr auto id = 4; };
    struct JoyControlChanged    {static constexpr auto id = 5; };
    struct MotorOnEvent         {static constexpr auto id = 6; };
    struct MotorOffEvent        {static constexpr auto id = 7; };
    // clang-format on

  private:
    // pollers
    void sdkPoll();
    void rcPoll();

  private:
    // guard function
    bool requestEnterSDK();
    bool isSDK();
    bool requestEnterRC();

  private:
    // sub functions for guard
    bool haveLB2();

  private:
    RC*  hostRC;
    bool enter;
    bool exit;

  private:
#ifdef HAVE_DJIOSDK_PRIVATE
    //! @control authority callback
    void obtainControlCallback(const actionlib::SimpleClientGoalState& state,
                               const dji_sdk::AuthorityResultConstPtr& result);

    void releaseControlCallback(const actionlib::SimpleClientGoalState& state,
                                const dji_sdk::AuthorityResultConstPtr& result);

    void motorOnCallback(const actionlib::SimpleClientGoalState&   state,
                         const dji_sdk::MotorSwitchResultConstPtr& result);
    void motorOffCallback(const actionlib::SimpleClientGoalState&   state,
                          const dji_sdk::MotorSwitchResultConstPtr& result);

    void landingCallback(const actionlib::SimpleClientGoalState&  state,
                         const dji_sdk::FlightTaskResultConstPtr& result);
#endif

  private:
    void obtainControl();
    void releaseControl();

    void publishCommands();

  public:
    auto operator()() noexcept
    {
      using namespace sml;

      auto sdk_enterFinish = event<PollEvent>[([this] {
        if (hostRC->simulation)
          return true;
        if (enter)
        {
          if (isSDK())
          {
            return true;
          }
          else
          {
            ROS_INFO("Re-obtain control");
            obtainControl();
            return false;
          }
        }
        else
        {
          return false;
        }
      })];

      auto rc_enterFinish = event<PollEvent>[([this] {
        if (exit)
        {
          if (isSDK())
          {
            ROS_INFO("Re-release control");
            return false;
          }
          else
          {
            if (!haveLB2())
              return true;
            else if (hostRC->osdk.buttons[BUTTON_MODE] == 1)
              return false;
            else
              return true;
          }
        }
        else
        {
          ROS_INFO("Wait for release control");
          return false;
        }
      })];

      auto sdkpoll = event<PollEvent>[([this] {
        bool ans = requestEnterRC();
        sdkPoll();
        if (!ans)
        {
          //          ROS_INFO("keep in SDK");
        }
        return ans;
      })];

      auto enterSDK = sml::on_entry<_> / [this] {
        this->hostRC->eventQueue.push(EVENT_ENTERSDK);
        std::cout << "Enter SDK mode" << std::endl;
      };

      auto exitSDK = sml::on_exit<_> / [this] {
        this->hostRC->eventQueue.push(EVENT_EXITSDK);
        std::cout << "Exit SDK mode" << std::endl;
      };

      auto motorOn = event<MotorOnEvent> / [this] {
        this->hostRC->eventQueue.push(RC::EVENT_REQUEST_TAKEOFF);
        if (this->hostRC->simulation)
          return;
#ifdef HAVE_DJIOSDK_PRIVATE
        //! @todo implement
        dji_sdk::MotorSwitchGoal goal;
        goal.startMotor = true;
        this->hostRC->motorSwitchClient.sendGoal(
          goal, boost::bind(&RC::Logic::motorOnCallback, this, _1, _2),
          MotorSwitchClient::SimpleActiveCallback(),
          MotorSwitchClient::SimpleFeedbackCallback());
#endif
      };

      auto motorOff = event<MotorOffEvent> / [this] {
        this->hostRC->eventQueue.push(RC::EVENT_REQUEST_LANDING);
        if (this->hostRC->simulation)
          return;
#ifdef HAVE_DJIOSDK_PRIVATE
        //! @todo implement
        dji_sdk::MotorSwitchGoal goal;
        goal.startMotor = false;
        this->hostRC->motorSwitchClient.sendGoal(
          goal, boost::bind(&RC::Logic::motorOffCallback, this, _1, _2),
          MotorSwitchClient::SimpleActiveCallback(),
          MotorSwitchClient::SimpleFeedbackCallback());
#endif
      };

      auto landing = event<MotorOffEvent> / [this] {
        //! @todo implement
        this->hostRC->eventQueue.push(RC::EVENT_REQUEST_LANDING);
        if (this->hostRC->simulation)
          return;
#ifdef HAVE_DJIOSDK_PRIVATE
        dji_sdk::FlightTaskGoal goal;
        goal.task = dji_sdk::FlightTaskGoal::TASK_LANDING;
        this->hostRC->flightTaskClient.sendGoal(
          goal, boost::bind(&RC::Logic::landingCallback, this, _1, _2),
          FlightTaskClient::SimpleActiveCallback(),
          FlightTaskClient::SimpleFeedbackCallback());
#endif
      };

      auto rcpoll = event<PollEvent>[([this] {
        rcPoll();
        return requestEnterSDK();
      })];

      //! @todo maybe add some logic
      //      auto joyControl = event<JoyControlChanged> /

      // clang-format off
      return make_transition_table(
            "rc"_s        <= *"init"_s
           ,"sdk_enter"_s <=  "rc"_s          + rcpoll
           ,"sdk_exit"_s  <=  "sdk"_s         + sdkpoll
           ,"sdk_enter"_s <=  "rc"_s          + event<RequestEnterSDKEvent>
           ,"sdk"_s       <=  "rc"_s          + event<AlreadyEnterSDKEvent>
           ,"sdk"_s       <=  "sdk_enter"_s   + sdk_enterFinish
           ,"sdk_exit"_s  <=  "sdk"_s         + event<RequestEnterRCEvent>
           ,"rc"_s        <=  "sdk"_s         + event<AlreadyEnterRCEvent>
           ,"rc"_s        <=  "sdk_exit"_s    + rc_enterFinish
            //! @note entry and exit
           ,"sdk_enter"_s + sml::on_entry<_> / [this]{obtainControl();}
           ,"sdk_exit"_s  + sml::on_entry<_> / [this]{releaseControl();}
           ,"sdk"_s + enterSDK
           ,"sdk"_s + exitSDK

           //! @note things to do in SDK state
           ,"sdk"_s + motorOn
           ,"sdk"_s + landing
          );
      // clang-format on
    }

  public:
    void setHostRC(RC* value);
  }; // class Logic

  typedef sml::sm<RC::Logic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

private:
  //! @note no default constructor
  RC();

public:
  RC(ros::NodeHandle& nh, OperationalLogic::StateMachine* sm);
  ~RC();

public:
  // importaer
  void fromJoy(const sensor_msgs::Joy::ConstPtr& data);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& data);

#ifdef HAVE_DJIOSDK_PRIVATE
  void fromOSDK(const sensor_msgs::Joy::ConstPtr& data);
  void sdkControlDevice(const dji_sdk::ControlDevice::ConstPtr& data);
#endif

private:
  //! @note intrenal functions

private:
  bool isOSDKRCLost(); // cannot achive for osdk 3.3.1 and its F.W.
  bool isJoyRCLost();

public:
  EVENT process();

private:
  std::queue<EVENT> eventQueue;

private:
  //! @note data session
  Data               osdk;
  Data               joy;
  bool               sdkControlling;
  bool               lastSwitchLeft;
  bool               lastSwitchRight;
  bool               simulation;
  nav_msgs::Odometry odom;
  double             x0;
  double             y0;
  double             z0;
  double             zOffset;
  bool               takeoffHover;
  bool               hasYawControl;
  double             horizontalGain;
  double             verticalGain;
  double             yawGain;

  int currentControlling;
  int id;

private:
  std::string missionCurrent;
  std::string missionA;
  std::string missionB;
  std::string missionX;
  std::string missionY;

private:
  //! @note ros session
  ros::Subscriber sub_dji;
  ros::Subscriber sub_joy;
  ros::Subscriber sub_ctrl_dev;
  ros::Subscriber sub_odom;

  ros::Publisher mission_publisher;
  ros::Publisher position_cmd_publisher;
  ros::Publisher attitude_cmd_publisher;
  ros::Publisher marker_color_publisher;
  ros::Publisher enable_control_publisher;
  ros::Publisher internal_joy_publisher;

private:
  //! @note command session

private:
  std::array<int, JOY_ALL> joyMap;

private:
  //! @note rc state machine
  OperationalLogic::StateMachine* logic;

  SMLLogger     logger;
  StateMachine* statemachine;

  //! @note dji_sdk related wrapper
private:
#ifdef HAVE_DJIOSDK_PRIVATE
  typedef actionlib::SimpleActionClient<dji_sdk::AuthorityAction>
    AuthorityClient;
  typedef actionlib::SimpleActionClient<dji_sdk::MotorSwitchAction>
    MotorSwitchClient;
  typedef actionlib::SimpleActionClient<dji_sdk::FlightTaskAction>
    FlightTaskClient;
#endif

private:
#ifdef HAVE_DJIOSDK_PRIVATE
  AuthorityClient   authorityClient;
  MotorSwitchClient motorSwitchClient;
  FlightTaskClient  flightTaskClient;
#endif

  void publish_mission(int mission);

public:
// callbacks
#ifdef HAVE_DJIOSDK_PRIVATE
  void authorityDone(
    const actionlib::SimpleClientGoalState& state,
    const dji_sdk::AuthorityResultConstPtr& result); // for debug
#endif

public:
  // getters
  int  getId() const;
  Data getInput() const;

public:
  // setters
  void setID(int value);
  std::string getMissionCurrent() const;
  void setMissionCurrent(const std::string& value);
};

} // namespace mocka

#endif // RC_HPP
