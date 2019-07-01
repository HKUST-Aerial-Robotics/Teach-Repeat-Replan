#ifndef REMOTENODE_HPP
#define REMOTENODE_HPP

#include <ros/ros.h>

#include "eventspinner.hpp"
#include "mission.hpp"
#include "statemachine.hpp"

#include "link.hpp"

namespace mocka
{

class Self
{
public:
  Self(std::shared_ptr<Link> l);

public:
  void poll();

  bool addInput(std::string&& name, LinkItemBase* clause);
  bool addOutput(std::string&& name, LinkItemBase* clause);

  void setNetworkSize(int value);

public:
  //! @note for test only
  std::shared_ptr<Link> getLink() const;

private:
  int  syncLinkage();
  bool readMAC();

private:
  std::shared_ptr<Link> link;

  int networkSize;
  int syncedFellow;

}; // RemoteNode

} // namespace  mocka

#endif // REMOTENODE_HPP
