#include <gtest/gtest.h>

#include <algorithm>
#include <ctime>
#include "cascadecontrol.hpp"

TEST(cascade_test, translation_pos_spin)
{
  mocka::CascadeController ctrl;

  mocka::State in;
  mocka::State feed;

  in.position   = Eigen::Vector3d(1, 1, 1);
  feed.position = Eigen::Vector3d(0, 0, 0);

  ctrl.setFeeds(feed);
  ctrl.setInput(in);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput().position, Eigen::Vector3d(1, 1, 1));
  feed.position = Eigen::Vector3d(0.5, 0.5, 0.5);
  ctrl.setFeeds(feed);
  ctrl.setInput(in);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput().position, Eigen::Vector3d(0.5, 0.5, 0.5));
}

TEST(cascade_test, translation_vel_spin)
{
  mocka::CascadeController ctrl;

  mocka::State in;
  mocka::State feed;

  in.position   = Eigen::Vector3d(0, 0, 0);
  in.velocity   = Eigen::Vector3d(1, 1, 1);
  feed.position = Eigen::Vector3d(0, 0, 0);
  feed.velocity = Eigen::Vector3d(0, 0, 0);

  ctrl.setFeeds(feed);
  ctrl.setInput(in);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput().velocity, Eigen::Vector3d(1, 1, 1));
  feed.velocity = Eigen::Vector3d(0.5, 0.5, 0.5);
  ctrl.setFeeds(feed);
  ctrl.setInput(in);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput().velocity, Eigen::Vector3d(0.5, 0.5, 0.5));
}

TEST(cascade_test, translation_acc_spin)
{
  mocka::CascadeController ctrl;

  mocka::State in;
  mocka::State feed;

  in.position       = Eigen::Vector3d(0, 0, 0);
  in.velocity       = Eigen::Vector3d(0, 0, 0);
  in.acceleration   = Eigen::Vector3d(1, 1, 1);
  feed.position     = Eigen::Vector3d(0, 0, 0);
  feed.velocity     = Eigen::Vector3d(0, 0, 0);
  feed.acceleration = Eigen::Vector3d(0, 0, 0);

  ctrl.setFeeds(feed);
  ctrl.setInput(in);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput().acceleration, Eigen::Vector3d(1, 1, 1));
  feed.acceleration = Eigen::Vector3d(0.5, 0.5, 0.5);
  ctrl.setFeeds(feed);
  ctrl.setInput(in);
  ctrl.spinOnce();
  EXPECT_EQ(ctrl.getOutput().acceleration, Eigen::Vector3d(0.5, 0.5, 0.5));
}

int
main(int argc, char* argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
