#ifndef ROUTER_HPP
#define ROUTER_HPP

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "mission_template.hpp"

class RouterNode : public MissionTemplateNode
{
public:
  RouterNode();
  virtual ~RouterNode();

public:
  Eigen::Vector3d getInitialPosition() const;

public:
  void onInit();

protected:
  virtual void run(double dt);
  virtual void clean();
  virtual bool start();

private:
  void listenerCallback(const quadrotor_msgs::PositionCommand::ConstPtr& ptr);

private:
  Eigen::Vector3d initialPosition;

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;

  std::string     input;
  ros::Subscriber listener;

  bool hasInput;
};

#endif // ROUTER_HPP
