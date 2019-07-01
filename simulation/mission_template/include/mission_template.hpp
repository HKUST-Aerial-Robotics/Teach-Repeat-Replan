#ifndef MISSION_TEMPLATE_HPP
#define MISSION_TEMPLATE_HPP

#include <ros/ros.h>

#include <nodelet/nodelet.h>

#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h> //! @note for dji_sdk controller command
#include <std_msgs/String.h>

#include "statemachine.hpp"

#include <Eigen/Eigen>
#include <chrono>
#include <queue>

#include <nav_msgs/Path.h>

class MissionTemplateNode : public nodelet::Nodelet
{
public:
  class Logic
  {
  public:
    // clang-format off
    struct RuntimeEvent   {int id = 0;};
    struct PollEvent      {static constexpr auto id = 0;};
    struct StartEvent     {static constexpr auto id = 1;};
    struct StopEvent      {static constexpr auto id = 2;};
    struct InitEvent      {static constexpr auto id = 3;};
    // clang-format on
  public:
    //! @note state machine
    auto operator()() noexcept
    {
      using namespace boost::sml;

      auto runPoll = event<PollEvent> / [this] {
        std::chrono::duration<double> diff =
          std::chrono::system_clock::now() - host->startTime;
        double dt =
          std::chrono::duration_cast<std::chrono::milliseconds>(diff).count() /
          1000.0;
        host->run(dt);
      };

      auto restart = sml::on_entry<_> / [this] { host->clean(); };

      auto clear = sml::on_entry<_> / [this] { host->clean(); };

      // clang-format off
      return make_transition_table(
             "idle"_s     <= *"init"_s    + event<InitEvent>
           , "start"_s    <=  "idle"_s    + event<StartEvent>
           , "restart"_s  <=  "running"_s + event<StartEvent>
           , "idle"_s     <=  "running"_s + event<StopEvent>
           , "running"_s  <=  "restart"_s + event<PollEvent> [([this]{return host->start();})]
           , "running"_s  <=  "start"_s   + event<PollEvent> [([this]{return host->start();})]
           , "idle"_s     <=  "restart"_s + event<StopEvent>
           , "idle"_s     <=  "start"_s   + event<StopEvent>
           , "restart"_s  + restart
           , "idle"_s     + clear
           , "running"_s  + runPoll
            );
      // clang-format on
    }

  public:
    MissionTemplateNode* getHost() const;

  public:
    void setHost(MissionTemplateNode* value);

  private:
    MissionTemplateNode* host;
  }; // class Logic

  typedef boost::sml::sm<MissionTemplateNode::Logic,
                         boost::sml::thread_safe<std::recursive_mutex>,
                         boost::sml::logger<mocka::SMLLogger> >
    StateMachine;

public:
  MissionTemplateNode();
  virtual ~MissionTemplateNode();

public:
  virtual void onInit();

public:
  //! @note state machine control
  void startMission();
  void stopMission();
  void resetTime();
  void resetPath();

protected:
  //! @note virturl functions for polling
  virtual void run(double dt);
  virtual void clean();
  virtual bool start();
  virtual void poll();
  virtual void joy(const sensor_msgs::Joy::ConstPtr& data);

  virtual void publishCommand(const Eigen::Vector3d& position_des,
                              const Eigen::Vector3d& velocity_des,
                              const Eigen::Vector3d& acceleration_des);

public:
  //! @note getters
  std::string getName() const;
  uint32_t    getPriority() const;

  std::queue<Eigen::Vector3d>    getPosition() const;
  std::queue<Eigen::Vector3d>    getVelocity() const;
  std::queue<Eigen::Quaterniond> getQuaternion() const;

public:
  //! @note setters
  void setName(const std::string& value);
  void setPriority(const uint32_t& value);

public:
  //! @note Advanced APIs
  Eigen::Vector3d readAveragePosition(int maxRange = 50);
  Eigen::Vector3d readAverageVelocity(int maxRange = 50);
  Eigen::Quaterniond readAverageQuaternion(int maxRange = 50);

protected:
  std::string      name;
  uint32_t         priority; //! @note useless sofar
  mocka::SMLLogger logger;
  StateMachine*    statemachine;
  nav_msgs::Path   path;

  sensor_msgs::Joy lastJoy;

private:
  //! @note callbacks
  void poller_callback(const std_msgs::String::ConstPtr& data);
  void enable_callback(const std_msgs::String::ConstPtr& data);
  void joy_callback(const sensor_msgs::Joy::ConstPtr& data);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& data);

protected:
  ros::Publisher position_command_pub;
  ros::Publisher path_pub;
  ros::Publisher state_pub; //! @note useless sofar

private:
  ros::Subscriber mission_poller_sub;
  ros::Subscriber mission_enable_sub;
  ros::Subscriber joy_sub;
  ros::Subscriber odom_sub;

private:
  Eigen::Vector3d    lastPosition;
  Eigen::Vector3d    lastVelocity;
  Eigen::Quaterniond lastQuaternion;

  static constexpr int           maxStorage = 10;
  std::queue<Eigen::Vector3d>    position;
  std::queue<Eigen::Vector3d>    velocity;
  std::queue<Eigen::Quaterniond> quaternion;

  typedef std::chrono::time_point<std::chrono::system_clock> TimePoint;

  TimePoint startTime;
};

#endif // MISSION_TEMPLATE_HPP
