#include <iostream>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sys/time.h>
#include <string>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <quadrotor_msgs/PositionCommand.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Joy.h>
#include "tf/transform_broadcaster.h"

using namespace Eigen;
using namespace std;


ros::Publisher current_odom_pub;

ros::Subscriber joy_sub;

tf::TransformBroadcaster*  broadcaster;

double start_x = 0.0, start_y =0.0, start_z = 1.0, min_z = 0.0, max_z = 1.0;
double start_yaw = 0.0;
Vector3d uav_position = Vector3d::Zero();
double uav_yaw = 0.0;

double roll = 0.0, pitch = 0.0, yaw = 0.0, thr = 0.0;

double hori_scale = 0.03, vert_scale = 0.02, yaw_scale = 0.01;


void get_des_from_js(Vector3d& des_p, double& des_yaw, double& droll, double& dpitch, double& dyaw, double& dthr)
{
  Vector3d p_c(
      dpitch * hori_scale,
      droll * hori_scale,
      dthr   * vert_scale);

  // assume that protocal's q is b^q_g.
  Matrix3d wRc;
  wRc << cos(uav_yaw), -sin(uav_yaw), 0.0,
         sin(uav_yaw), cos(uav_yaw), 0.0,
         0.0, 0.0, 1.0; 
  Vector3d v_w = wRc * p_c;

  des_p = v_w + uav_position;

  des_p.z() = min(max(min_z, des_p.z()), max_z);

  des_yaw = dyaw * yaw_scale + uav_yaw;

  while(des_yaw > M_PI) des_yaw -= (2*M_PI);
  while(des_yaw < -M_PI) des_yaw += (2*M_PI); 
}

void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if (msg->buttons.at(7)){
    uav_position << start_x, start_y, start_z;
    uav_yaw = start_yaw;
  }
  roll = msg->axes[3]; pitch = msg->axes[4]; yaw = msg->axes[0]; thr = msg->axes[1];

  roll = roll > -0.05 && roll < 0.05 ? 0 : roll;
  pitch = pitch > -0.05 && pitch < 0.05 ? 0 : pitch;
  yaw = yaw > -0.05 && yaw < 0.05 ? 0 : yaw;
  thr = thr > -0.05 && thr < 0.05 ? 0 : thr;
}

void pub_odom(){
  Matrix3d wRc;
  wRc << cos(uav_yaw), -sin(uav_yaw), 0.0,
         sin(uav_yaw), cos(uav_yaw), 0.0,
         0.0, 0.0, 1.0;

  Quaterniond wQc(wRc);

  nav_msgs::Odometry odom;
  odom.header.frame_id = "/world";
  odom.header.stamp = ros::Time::now();
  odom.pose.pose.position.x = uav_position.x();
  odom.pose.pose.position.y = uav_position.y();
  odom.pose.pose.position.z = uav_position.z(); 

  odom.pose.pose.orientation.w = wQc.w();
  odom.pose.pose.orientation.x = wQc.x();
  odom.pose.pose.orientation.y = wQc.y();
  odom.pose.pose.orientation.z = wQc.z();

  current_odom_pub.publish(odom);

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(uav_position.x(), uav_position.y(), uav_position.z()));
  transform.setRotation(tf::Quaternion(wQc.x(), wQc.y(), wQc.z(), wQc.w()));
  broadcaster->sendTransform(tf::StampedTransform(
  transform, ros::Time::now(),
  string("/world"), "/drone"));
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "traj_generator");
	ros::NodeHandle nh("~");

    nh.param("start_x", start_x, 0.0);
    nh.param("start_y", start_y, 0.0);
    nh.param("start_z", start_z, 1.0);
    nh.param("start_yaw", start_yaw, 0.0);

    nh.param("min_z", min_z, 0.0);
    nh.param("max_z", max_z, 2.0);

    nh.param("hori_scale", hori_scale, 0.03);
    nh.param("vert_scale", vert_scale, 0.02);
    nh.param("yaw_scale", yaw_scale, 0.01);

    uav_position << start_x, start_y, start_z;
    uav_yaw      =  start_yaw;
    
    broadcaster = new tf::TransformBroadcaster();

    current_odom_pub = nh.advertise<nav_msgs::Odometry>("uav_odom_sim" , 1);

    joy_sub = nh.subscribe( "joy", 10, joy_callback);

    ros::Rate r(100);
	while(ros::ok()){
        ros::spinOnce();
        get_des_from_js(uav_position, uav_yaw, roll, pitch, yaw, thr);
        pub_odom();
        r.sleep();
    }

	return 0;
}
