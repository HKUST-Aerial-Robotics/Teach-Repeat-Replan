#include <gtest/gtest.h>

#include "dataio.hpp"

#include <sensor_msgs/Joy.h>

TEST(mac_address, feature_test)
{
  mocka::DataIO io;
}

TEST(init, feature_test)
{
  mocka::DataIO io;

  std::shared_ptr<mocka::Link> sl = io.getSelf().getLink();

  EXPECT_NE(reinterpret_cast<size_t>(sl->getInput("global_rc")), 0);

  sensor_msgs::Joy& j =
    sl->getInput<mocka::LinkItem<sensor_msgs::Joy> >("global_rc")->get<0>();

  j.axes.push_back(9.99);
  EXPECT_FLOAT_EQ(sl->getInput<mocka::LinkItem<sensor_msgs::Joy> >("global_rc")
                    ->get<0>()
                    .axes[0],
                  9.99);

  std::vector<int8_t> d;
  d.push_back(0xAA);
  io.sendMetaData(d);
  io.sendMetaData(d);
  io.poll(); // this is important for the subscriber to receive data
  std::cout << "poll" << std::endl;
  io.poll();
}

int
main(int argc, char* argv[])
{
  ros::init(argc, argv, "iotest");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
