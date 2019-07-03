#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud, nav_msgs::Path, nav_msgs::Odometry> exact_policy;

#include <surfel_map.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surfel_fusion");
  ros::NodeHandle nh("~");

  SurfelMap surfel_map(nh);

  ros::Subscriber sub_image = nh.subscribe("image", 1, &SurfelMap::image_input, &surfel_map);
  ros::Subscriber sub_depth = nh.subscribe("depth", 1, &SurfelMap::depth_input, &surfel_map);
  ros::Subscriber sub_path = nh.subscribe("loop_path", 1, &SurfelMap::path_input, &surfel_map);
  ros::Subscriber sub_extrinsic_pose = nh.subscribe("extrinsic_pose", 1, &SurfelMap::extrinsic_input, &surfel_map);

  ros::Rate r(100);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
    surfel_map.publish_all_pointcloud(ros::Time::now());
  }

  return EXIT_SUCCESS;
}