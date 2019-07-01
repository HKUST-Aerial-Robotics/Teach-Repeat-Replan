#ifndef DATAIO_HPP
#define DATAIO_HPP

#include <map>

#include <std_msgs/ByteMultiArray.h>

#include "link.h"
#include "monitor.hpp"
#include "self.hpp"

namespace mocka
{

class DataIO
{
public:
  DataIO();

public:
  Self&    getSelf();
  Monitor& getMonitor();

public:
  void poll();
  void sendMetaData(std::vector<int8_t>& vdata);

protected:
  void metaCallback(const std_msgs::ByteMultiArray::ConstPtr& data);

private:
  Monitor monitor;
  Self    self;

  std::vector<std::shared_ptr<Link> > links;

  std::map<std::string, ros::Publisher>  linkPublisherPool;
  std::map<std::string, ros::Subscriber> linkSubscriberPool;

  ros::NodeHandle nh;
};

} // namespace mocka
#endif // DATAIO_HPP
