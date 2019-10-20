/** @file main.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  Main function for ROS Node
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#include <dji_sdk/dji_sdk_node.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "dji_sdk");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ROS_WARN("main.cpp");

  DJISDKNode* dji_sdk_node = new DJISDKNode(nh, nh_private);

  ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();

  delete dji_sdk_node;
  dji_sdk_node = NULL;

  return 0;
}
