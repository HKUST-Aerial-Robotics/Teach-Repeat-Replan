#include "boost/sml/utility/dispatch_table.hpp"
#include <boost/sml.hpp>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <ros/ros.h>

#include "statemap.hpp"

#include "tf/tf.h"
#include "tf/transform_broadcaster.h"

tf::TransformBroadcaster* tfBroadcaster;

namespace sml = boost::sml;

namespace mocka
{
template <>
void
OperationalLogic::addEvent(const RC::EVENT& e)
{
  switch (e)
  {
    case RC::EVENT_ENTERSDK:
      fifo.push(RegisterSDK::id);
      break;
    case RC::EVENT_EXITSDK:
      fifo.push(QuitSDK::id);
      break;
    case RC::EVENT_REQUEST_TAKEOFF:
      fifo.push(TakeOffRunning::id);
      break;
    case RC::EVENT_REQUEST_LANDING:
      fifo.push(LandingRunning::id);
      break;
    case RC::EVENT_REQUEST_ENTER_MISSION:
      fifo.push(REQEnterMission::id);
      break;
    case RC::EVENT_REQUEST_EXIT_MISSION:
      fifo.push(REQQuitMission::id);
      break;
    default:
      break;
  }
}
} // namespace mocka

int
main(int argc, char** argv)
{
  using namespace mocka;
  using namespace sml;

  typedef OperationalLogic::StateMachine StateMachine;

  ros::init(argc, argv, "state_machine");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  SMLLogger logger;

  StateMachine      sm{ logger };
  OperationalLogic* logic = &sm.getInfo();
  logic->setExternalMission(
    std::shared_ptr<ExternalMission>(new ExternalMission(nh_private)));

  ros::Subscriber flight_status_sub = nh.subscribe(
    "sdk/flight_status", 10, &OperationalLogic::flight_status_callback, logic,
    ros::TransportHints().tcpNoDelay());

  RC  rcinput(nh_private, &sm);
  int id;
  int rate;
  nh_private.param("drone_id", id, 0);
  nh_private.param("rate", rate, 100);
  rcinput.setID(id);
  auto dispatch_event =
    sml::utility::make_dispatch_table<OperationalLogic::RuntimeEvent, 1, 20>(
      sm);
  ros::Rate r(rate);

  //  tfBroadcaster = new tf::TransformBroadcaster();

  //  tf::StampedTransform transform;

  //  transform.setOrigin(tf::Vector3(0, 0, 0));
  //  transform.setRotation(tf::Quaternion(0, 0, 0, 1));

  //  tfBroadcaster->sendTransform(tf::StampedTransform(
  //    transform, ros::Time::now(), "/map", "/local_map_" +
  //    std::to_string(id)));

  while (ros::ok())
  {
    logic->addEvent(rcinput.process());
    int id  = logic->getFIFO()->pop();
    int len = logic->getFIFO()->length();

    OperationalLogic::RuntimeEvent e{ id };

    if (len > 500)
      ROS_WARN("Too many event in the queue to process %d", len);

    if (e.id)
      dispatch_event(e, e.id);
    else
      sm.process_event(OperationalLogic::EmptyEvent{});

    ros::spinOnce();
    r.sleep();
  }

  //  delete tfBroadcaster;
}
