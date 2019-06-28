#ifndef EXTERNAL_MISSION_HPP
#define EXTERNAL_MISSION_HPP

#include <ros/ros.h>

namespace mocka
{

class ExternalMission
{
public:
  ExternalMission(ros::NodeHandle& nh);

private:
  //! @todo make it private
public:
  ExternalMission();

public:
  void run();
  void pend();

private:
  ros::Publisher external_process_pub;
};

} // namespace mocka

#endif // EXTERNAL_MISSION_HPP
