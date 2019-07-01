#ifndef STATEMACHINE_HPP
#define STATEMACHINE_HPP

#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>

#include "boost/sml.hpp"
#include "mission.hpp"
#include "safefifo.hpp"

#include "stateitem.hpp"

#include <std_msgs/UInt8.h>

namespace sml = boost::sml;

namespace mocka
{

struct SMLLogger
{
  template <class SM, class TEvent>
  void log_process_event(const TEvent&)
  { /*
     if (boost::starts_with(sml::aux::get_type_name<TEvent>(), "boost"))
       printf("[process_event] Internal\n");
     else
       printf("[process_event] %s\n", sml::aux::get_type_name<TEvent>());
  */
  }

  template <class SM, class TGuard, class TEvent>
  void log_guard(const TGuard&, const TEvent&, bool result)
  { /*
     printf("[guard] %s %s\n", sml::aux::get_type_name<TGuard>(),
            (result ? "[OK]" : "[Reject]"));*/
  }

  template <class SM, class TAction, class TEvent>
  void log_action(const TAction&, const TEvent&)
  { /*
     printf("[action]\n");*/
  }

  template <class SM, class TSrcState, class TDstState>
  void log_state_change(const TSrcState& src, const TDstState& dst)
  {
    printf("[transition] %s -> %s\n", src.c_str(), dst.c_str());
  }
};

class OperationalLogic
{
public:
  typedef sml::sm<OperationalLogic, sml::thread_safe<std::recursive_mutex>,
                  sml::logger<SMLLogger> >
    StateMachine;

public:
  OperationalLogic();
  ~OperationalLogic();

public:
  //! @note event list
  // clang-format off
  struct RuntimeEvent     {int id = 0;};
  struct EmptyEvent       {static constexpr auto id = 0;};
  struct RegisterSDK      {static constexpr auto id = 1;};
  struct TakeOffRunning   {static constexpr auto id = 2;};
  struct LandingRunning   {static constexpr auto id = 3;};
  struct REQEnterRC       {static constexpr auto id = 4;};
  struct REQQuitRC        {static constexpr auto id = 5;};
  struct REQEnterMission  {static constexpr auto id = 6;};
  struct REQQuitMission   {static constexpr auto id = 7;};
  struct BatteryLow       {static constexpr auto id = 8;};
  struct PowerOn          {static constexpr auto id = 9;};
  struct Sleep            {static constexpr auto id = 10;};
  struct PowerOff         {static constexpr auto id = 11;};
  struct ShutDown         {static constexpr auto id = 12;};
  struct CrashDetected    {static constexpr auto id = 13;};
  struct QuitSDK          {static constexpr auto id = 14;};

  // clang-format on

public:
  //! @note pollers
  void groundIdlePoll();
  void skyIdlePoll();
  void rcControlPoll();
  void missionPoll();
  void rthPoll();
  void powerOnPoll();
  void sleepPoll();
  void powerOffPoll();
  void crashPoll();

public:
  //! @note Guards
  bool isOnLand();

public:
  //! @note Actions

public:
  auto operator()() noexcept
  {
    //! @attention this state machine only maintain statuses, it won't do
    //! anything which will have actural effect, those dirty jobs would be done
    //! by other sub-module which is loosely coupled with this state machine.
    //!
    //! @note in c++1z/c++1y this function cannote move to .cpp file,
    //! this is a c++1z/c++1y feature, if you want to move this function to
    //! .cpp file, you need to declear the return value without auto
    //! since the boost::sml use auto in make_transition_table,
    //! it will be a b bit hard to find out how to achive this.
    using namespace sml;

    auto take_off = sml::event<TakeOffRunning> / [this] { onLand = false; };
    auto landing = sml::event<LandingRunning> / [this] { onLand = true; };
    auto crashEntry = sml::on_entry<_> / [this] {
      crashTime = std::chrono::system_clock::now();
      std::time_t now;
      std::time(&now);
      std::cout << "Drone " << name << "crashed @ " << std::ctime(&now)
                << std::endl;
    };

    // clang-format off
    return make_transition_table(
          /*! @brief
           *  state list:
           *  "power_on"_s
           *  "power_off"_s
           *  "sleep"_s
           *  "crashed"_s
           *  "ground_idle"_s
           *  "sky_idle"_s
           *  "manual_control"_s
           *    This state only affect the DJI RC connected situation, totally manual control by RC
           *  "mission"_s
           *  "rth"_s
           *    this is still under SDK mode but just controller by joystick
           *
           * */


          //! @note transition
          "ground_idle"_s     <=  "sky_idle"_s        + landing                     ,
          "ground_idle"_s     <=  "rth"_s             + landing                     ,
          "sky_idle"_s        <=  "ground_idle"_s     + take_off                    ,
          "manual_control"_s  <=  "ground_idle"_s     + event<REQEnterRC>           ,
          "manual_control"_s  <=  "sky_idle"_s        + event<REQEnterRC>           ,
          "mission"_s         <=  "sky_idle"_s        + event<REQEnterMission>      ,
          "mission"_s         <=  "ground_idle"_s     + event<REQEnterMission>      ,
          "rth"_s             <=  "sky_idle"_s        + event<BatteryLow>           ,
          "rth"_s             <=  "manual_control"_s  + event<BatteryLow>           ,
          "power_on"_s        <=  "sleep"_s           + event<PowerOn>              ,
          "sleep"_s           <=  "ground_idle"_s     + event<Sleep>                ,
          "power_off"_s       <=  "ground_idle"_s     + event<PowerOff>             ,
          "crashed"_s         <=  "ground_idle"_s     + event<CrashDetected> ,//! @note take off fail
          "crashed"_s         <=  "sky_idle"_s        + event<CrashDetected>        ,
          "crashed"_s         <=  "manual_control"_s  + event<CrashDetected>        ,
          "crashed"_s         <=  "mission"_s         + event<CrashDetected>        ,
          "crashed"_s         <=  "rth"_s             + event<CrashDetected>        ,
          X                   <=  "power_off"_s       + event<ShutDown>             ,

          //! @note quit SDK transition
          "power_off"_s       + event<QuitSDK> = "power_on"_s ,
          "sleep"_s           + event<QuitSDK> = "power_on"_s ,
          "crashed"_s         + event<QuitSDK> = "power_on"_s ,
          "ground_idle"_s     + event<QuitSDK> = "power_on"_s ,
          "sky_idle"_s        + event<QuitSDK> = "power_on"_s ,
          "manual_control"_s  + event<QuitSDK> = "power_on"_s ,
          "mission"_s         + event<QuitSDK> = "power_on"_s ,
          "rth"_s             + event<QuitSDK> = "power_on"_s
          //! @note actions

          //! @note Guarded transition
        ,*"power_on"_s       + event<RegisterSDK> [([this]{return isOnLand();})]    = "ground_idle"_s
        , "power_on"_s       + event<RegisterSDK> [([this]{return !isOnLand();})]   = "sky_idle"_s
        , "manual_control"_s + event<REQQuitRC> [([this]{return isOnLand();})]      = "ground_idle"_s
        , "manual_control"_s + event<REQQuitRC> [([this]{return !isOnLand();})]     = "sky_idle"_s
        , "manual_control"_s + take_off // stay in manual control state, no transition
        , "manual_control"_s + landing  // stay in manual control state, no transition

        , "mission"_s    + event<REQQuitMission> [([this]{return isOnLand();})]  = "ground_idle"_s
        , "mission"_s    + event<REQQuitMission> [([this]{return !isOnLand();})] = "sky_idle"_s
        , "mission"_s    + take_off  // stay in mission state, no transition
        , "mission"_s    + landing   // stay in mission state, no transition

          //! @note entry and exit
        , "ground_idle"_s + sml::on_entry<_>/[]{ std::cout << "ground_idle on entry" << std::endl; }
        , "crashed"_s + crashEntry
          //! @note poller
        , "ground_idle"_s     /[this]{groundIdlePoll();}
        , "sky_idle"_s        /[this]{skyIdlePoll();}
        , "manual_control"_s  /[this]{rcControlPoll();}
        , "mission"_s         /[this]{missionPoll();}
        , "rth"_s             /[this]{rthPoll();}
        , "power_on"_s        /[this]{powerOnPoll();}
        , "sleep"_s           /[this]{sleepPoll();}
        , "power_off"_s       /[this]{powerOffPoll();}
        , "crashed"_s         /[this]{crashPoll();}
          //! @note error handleing, still not nessary
//        , *"exceptions handling"_s + exception<_> / [] { std::cout << "generic exception caught" << std::endl; } = X
        );
    // clang-format on
  }

public:
  // getters
  //! @todo wait for boost::sml patch
  SafeFIFO<int>* getFIFO();

  bool getOnLand() const;

public:
  // setters
  void setOnLand(bool value);
  void setExternalMission(const std::shared_ptr<ExternalMission>& value);

public:
  template <typename T>
  void addEvent(const T&)
  {
    std::cout << "Adding unknown event ";
    fifo.push(EmptyEvent::id);
  }

public:
  // roswarrper
  void flight_status_callback(const std_msgs::UInt8::ConstPtr& flightstatus);

private:
  SafeFIFO<int>  fifo;
  MissionManager mission;

private:
  //! @note statuses
  typedef std::chrono::time_point<std::chrono::system_clock> Time;

  std::string name;
  bool        onLand;

  Time crashTime;
}; // OperationalLogic

} // namespace mocka
#endif // STATEMACHINE_HPP
