#include "statemachine.hpp"

#include <ctime>
#include <iostream>

#ifdef HAVE_DJIOSDK
#define HAVE_DJIOSDK_PRIVATE
#include <djiosdk/dji_status.hpp>
#endif

using namespace mocka;
OperationalLogic::OperationalLogic()
  : onLand(true)
{
}

OperationalLogic::~OperationalLogic()
{
}

void
OperationalLogic::groundIdlePoll()
{
  static int counter = 0;
  if (counter++ % 100 == 0)
    std::cout << "~~~~Ground Poll" << std::endl;
  mission.pend();
  //! @note ground idle state supported actions:
  //! 1 take off
  //! 2 reinit
  //! 3 upload mission
}

void
OperationalLogic::skyIdlePoll()
{
  static int counter = 0;
  if (counter++ % 100 == 0)
    std::cout << "~~~~Sky Poll" << std::endl;
  mission.pend();
  //! @note sky idle state supported actions:
  //! 1 landing
  //! 2 rth
  //! 3 manual control

  //! 4 upload mission
  //! 5 start mission
}

void
OperationalLogic::rcControlPoll()
{
  std::cout << "~~~~RC Poll" << std::endl;
  //! @note manual control state supported actions:
  //! 1 control
  //! 2 quit control
  //! 3 landing
}

void
OperationalLogic::missionPoll()
{
  static int counter = 0;
  if (counter++ % 100 == 0)
    std::cout << "~~~~Mission Poll" << std::endl;
  mission.poll();
}

void
OperationalLogic::rthPoll()
{
  std::cout << "~~~~RTH Poll" << std::endl;
}

void
OperationalLogic::powerOnPoll()
{
  //  std::cout << "~~~~Power on Poll" << std::endl;
}

void
OperationalLogic::sleepPoll()
{
  std::cout << "~~~~Sleep Poll" << std::endl;
}

void
OperationalLogic::powerOffPoll()
{
  std::cout << "~~~~Power off Poll" << std::endl;
  //! @todo
}

void
OperationalLogic::crashPoll()
{
  std::cout << "~~~~Crash Poll" << std::endl;
  Time now = std::chrono::system_clock::now();

  std::chrono::duration<double> timePast = now - crashTime;
  if (timePast.count() > 2)
  {
    std::cout << "crash rescue timeout" << std::endl;
  }
  else
  {
    //! @todo rescue
    std::cout << "rescue " << timePast.count() << std::endl;
  } //! @todo implement
}

bool
OperationalLogic::isOnLand()
{
  return onLand;
}

SafeFIFO<int>*
OperationalLogic::getFIFO()
{
  return &fifo;
}

bool
OperationalLogic::getOnLand() const
{
  return onLand;
}

void
OperationalLogic::setOnLand(bool value)
{
  onLand = value;
}

void
OperationalLogic::setExternalMission(
  const std::shared_ptr<ExternalMission>& value)
{
  mission.setExternalMission(value);
}

void
OperationalLogic::flight_status_callback(
  const std_msgs::UInt8::ConstPtr& flightstatus)
{
#ifdef HAVE_DJIOSDK_PRIVATE
  bool insky =
    flightstatus->data == DJI::OSDK::VehicleStatus::FlightStatus::IN_AIR
      ? true
      : false;

  if (insky == getOnLand())
    setOnLand(!insky);
#endif
}
