#ifndef GROUNDNODE_HPP
#define GROUNDNODE_HPP

#include <ros/ros.h>

#include "eventspinner.hpp"
#include "mission.hpp"
#include "statemachine.hpp"

#include "link.hpp"

namespace mocka
{

class Monitor
{
public:
  Monitor();

private:
  std::vector<std::shared_ptr<Link> > links;
}; // GroundNode

} // namespace mocka

#endif // GROUNDNODE_HPP
