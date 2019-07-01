#include "external_mission.hpp"

#include "std_msgs/String.h"

using namespace mocka;

ExternalMission::ExternalMission(ros::NodeHandle& nh)
{
  external_process_pub =
    nh.advertise<std_msgs::String>("mission/trigger", 10, true);
}

ExternalMission::ExternalMission()
{
}

void
ExternalMission::run()
{
  //  ROS_INFO("runnning external mission");
  std_msgs::String data;
  data.data = "run";
  external_process_pub.publish(data);
}

void
ExternalMission::pend()
{
  std_msgs::String data;
  data.data = "pend";
  external_process_pub.publish(data);
}
