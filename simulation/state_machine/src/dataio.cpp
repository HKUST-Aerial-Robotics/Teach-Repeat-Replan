#include "dataio.hpp"

#include <iostream>

using namespace mocka;

DataIO::DataIO()
  : self(std::shared_ptr<Link>(new Link()))
  , nh()
{
  auto l = self.getLink();

  links.push_back(l);

  linkPublisherPool["/swarm/meta"] =
    nh.advertise<std_msgs::ByteMultiArray>("/swarm/meta", 5);
  linkSubscriberPool["/swarm/meta"] =
    nh.subscribe("/swarm/meta", 10, &DataIO::metaCallback, this);
}

Self&
DataIO::getSelf()
{
  return self;
}

Monitor&
DataIO::getMonitor()
{
  return monitor;
}

void
DataIO::poll()
{
  ros::spinOnce();
}

void
DataIO::sendMetaData(std::vector<int8_t>& vdata)
{
  std_msgs::ByteMultiArray test;
  test.data.insert(test.data.end(), vdata.begin(), vdata.end());
  std::cout << "publish" << std::endl;
  linkPublisherPool["/swarm/meta"].publish(test);
}

void
DataIO::metaCallback(const std_msgs::ByteMultiArray::ConstPtr& data)
{
  std::cout << "get data " << std::hex << std::uppercase << std::setfill('0')
            << std::setw(2) << (int)(*(uint8_t*)(&data->data[0])) << std::endl;
}
