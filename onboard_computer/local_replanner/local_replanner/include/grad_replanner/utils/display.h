#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>

#include <stdlib.h>
#include <vector>

#include "grad_spline/data_type.h"
#include "grad_spline/non_uniform_bspline.h"
#include "grad_spline/uniform_bspline.h"

using std::vector;
// Eigen::VectorXd my_time;
// int point_num;

ros::Publisher setpoint_pub;
ros::Publisher traj_pub;
ros::Publisher traj_point_pub;
ros::Publisher va_pub;
ros::Publisher path_pub;
ros::Publisher visited_pub;

// visualize initial waypoint
void visualizeSetPoints(vector<Eigen::Vector3d> points) {
  // send them to rviz
  for (int i = 0; i < points.size(); ++i) {
    visualization_msgs::Marker p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();
    p.id = i;

    p.type = visualization_msgs::Marker::SPHERE;
    p.action = visualization_msgs::Marker::ADD;

    p.pose.position.x = points[i][0];
    p.pose.position.y = points[i][1];
    p.pose.position.z = points[i][2];
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    p.scale.x = p.scale.y = p.scale.z = 0.2;

    p.color.a = p.color.r = 1.0;
    p.color.g = p.color.b = 0.0;

    p.lifetime = ros::Duration(2000.0);

    setpoint_pub.publish(p);
    ros::Duration(0.001).sleep();

    // ROS_INFO_STREAM("publish set point");
  }
}

// visualize initial waypoint
void visualizePoints(Eigen::MatrixXd points) {
  // send them to rviz
  srand(ros::Time::now().toSec());
  double cr = rand() / double(RAND_MAX);
  double cg = rand() / double(RAND_MAX);
  double cb = rand() / double(RAND_MAX);
  for (int i = 0; i < points.rows(); ++i) {
    visualization_msgs::Marker p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();
    p.id = i;

    p.type = visualization_msgs::Marker::SPHERE;
    p.action = visualization_msgs::Marker::ADD;

    p.pose.position.x = points(i, 0);
    p.pose.position.y = points(i, 1);
    p.pose.position.z = points(i, 2);
    p.pose.orientation.w = 1;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;

    p.scale.x = p.scale.y = p.scale.z = 0.1;

    p.color.a = 1.0;
    p.color.r = cr;
    p.color.g = cg;
    p.color.b = cb;

    p.lifetime = ros::Duration(2000.0);

    setpoint_pub.publish(p);
    ros::Duration(0.0001).sleep();

    // ROS_INFO_STREAM("publish set point");
  }
}

void displayPathWithTime(Eigen::MatrixXd path) {
  ros::Time start = ros::Time::now();
  // start msg
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = -1.0;
  odom.pose.pose.position.y = -1.0;
  odom.pose.pose.position.z = -1.0;
  va_pub.publish(odom);

  // publish the whole trajectory
  for (int i = 1; i < int(path.cols()); ++i) {
    odom.header.frame_id = "world";
    odom.header.stamp = ros::Time(path(0, i));
    // odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = path(1, i);
    odom.pose.pose.position.y = path(2, i);
    odom.pose.pose.position.z = path(3, i);
    odom.twist.twist.linear.x =
        (path(1, i) - path(1, i - 1)) / (path(0, i) - path(0, i - 1));
    odom.twist.twist.linear.y =
        (path(2, i) - path(2, i - 1)) / (path(0, i) - path(0, i - 1));
    odom.twist.twist.linear.z =
        (path(3, i) - path(3, i - 1)) / (path(0, i) - path(0, i - 1));
    va_pub.publish(odom);
    ros::Duration(0.001).sleep();

    std::cout << "position:" << path.col(i).transpose() << std::endl;
    std::cout << "velocity:"
              << (path.col(i) - path.col(i - 1)).transpose() /
                     (path(0, i) - path(0, i - 1))
              << std::endl;
    std::cout << std::endl;
  }

  odom.header.frame_id = "world";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 1.0;
  odom.pose.pose.position.z = 1.0;
  va_pub.publish(odom);
}

void displayPath(vector<Eigen::Vector3d> path, double resolution) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.a = 1.0;
  mk.color.r = 1.0;
  mk.color.g = 0.0;
  mk.color.b = 0.0;

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  path_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

// color 1234, rgby, resolution = 0.05
void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution,
                          int color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  path_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.a = 1.0;

  if (color == 1) {
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;
  } else if (color == 2) {
    mk.color.g = 1.0;
    mk.color.r = 0.0;
    mk.color.b = 0.0;
  } else if (color == 3) {
    mk.color.b = 1.0;
    mk.color.g = 0.0;
    mk.color.r = 0.0;
  } else if (color == 4) {
    mk.color.g = 1.0;
    mk.color.r = 1.0;
    mk.color.b = 0.0;
  }

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  path_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void displayVisitedNodes(vector<GridNodePtr> nodes, double resolution) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::CUBE_LIST;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = 0;

  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.a = 0.1;
  mk.color.r = 0.0;
  mk.color.g = 1.0;
  mk.color.b = 0.0;

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(nodes.size()); i++) {
    Eigen::Vector3d coord = nodes[i]->coord;
    pt.x = coord(0);
    pt.y = coord(1);
    pt.z = coord(2);

    mk.points.push_back(pt);
  }

  visited_pub.publish(mk);
}

// color 1234, rgby, resolution = 0.05
void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution,
                          Eigen::Vector4d color, int id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = id;

  path_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++) {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  path_pub.publish(mk);
  ros::Duration(0.001).sleep();
}



#endif