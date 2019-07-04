#include <iostream>
#include <math.h>
#include <random>
#include <Eigen/Eigen>

#include <tf/tf.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace Eigen;

ros::Publisher _obs_pub, _obs_all_pub;
ros::Subscriber _odom_sub, _goal_sub;
sensor_msgs::PointCloud2 _cloudObs_ros;
sensor_msgs::PointCloud2 _cloudAlObs_ros;

nav_msgs::Odometry _odom;
geometry_msgs::PoseStamped _pose;
pcl::PointCloud<pcl::PointXYZ> cloudObsAll;

double _resolution;
double _w_l, _w_h, _h_l, _h_h;
double _max_z;
bool _has_odom = false;

void rcvOdometryCallback(const nav_msgs::Odometry odom);
void rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

void randomObsGenerator(double yaw, double click_x, double click_y, double click_z)
{  
   pcl::PointCloud<pcl::PointXYZ> cloudObs;

   if(click_z < -2.0)
   {  
      cloudObs.points.clear();
      cloudObs.width = cloudObs.points.size();
      cloudObs.height = 1;
      cloudObs.is_dense = true;

      cloudObsAll.points.clear();
      cloudObsAll.width = cloudObsAll.points.size();
      cloudObsAll.height = 1;
      cloudObsAll.is_dense = true;

      //ROS_WARN("Clear all clicked obstacles");

      pcl::toROSMsg(cloudObs,    _cloudObs_ros);
      pcl::toROSMsg(cloudObsAll, _cloudAlObs_ros);

      _cloudObs_ros.header.frame_id = "map";
      _cloudAlObs_ros.header.frame_id = "map";
      _obs_pub.publish(_cloudObs_ros);
      _obs_all_pub.publish(_cloudAlObs_ros);
      return;
   }
   else
   {  
      double width_l, width_h;
      width_l = _w_l; 
      width_h = _w_h;
      
      /*if(yaw > 0.0){
         width_l = _w_l * 3.0; 
         width_h = _w_h * 3.0;
      }
      else{
         width_l = _w_l; 
         width_h = _w_h;
      }*/

      random_device rd;
      default_random_engine eng(rd());
      uniform_real_distribution<double> rand_w = uniform_real_distribution<double>(width_l, width_h);
      uniform_real_distribution<double> rand_h = uniform_real_distribution<double>(_h_l, _h_h);

      // now we only generate a 
      //ROS_WARN("generate a random pillar");
      double w = rand_w(eng);

      double coord_x, coord_y;
      coord_x = floor(click_x/_resolution) * _resolution + _resolution / 2.0;
      coord_y = floor(click_y/_resolution) * _resolution + _resolution / 2.0;

      int widNum = ceil(w/_resolution);

      pcl::PointXYZ pt_random;
      for(int r = -widNum/2.0; r < widNum/2.0; r ++ )
      {
         for(int s = -widNum/2.0; s < widNum/2.0; s ++ )
         {
            double h = min(_max_z , click_z + rand_h(eng));  
            int heiNum = 2.0 * ceil(h/_resolution);

            for(int t = 0; t < heiNum; t ++ )
            {
               pt_random.x = coord_x + (r+0.0) * _resolution;
               pt_random.y = coord_y + (s+0.0) * _resolution;
               pt_random.z = (t+0.0) * _resolution / 2.0;
               cloudObs.points.push_back( pt_random );
               cloudObsAll.points.push_back( pt_random );
            }
         }
      }

      cloudObs.width = cloudObs.points.size();
      cloudObs.height = 1;
      cloudObs.is_dense = true;

      cloudObsAll.width = cloudObsAll.points.size();
      cloudObsAll.height = 1;
      cloudObsAll.is_dense = true;

      //ROS_WARN("Finished a random pillar ");

      pcl::toROSMsg(cloudObs,    _cloudObs_ros);
      pcl::toROSMsg(cloudObsAll, _cloudAlObs_ros);

      _cloudObs_ros.header.frame_id = "map";
      _cloudAlObs_ros.header.frame_id = "map";
      _obs_pub.publish(_cloudObs_ros);
      _obs_all_pub.publish(_cloudAlObs_ros);
   }
}

void rcvOdometryCallback(const nav_msgs::Odometry odom)
{
   _has_odom = true;
   _odom = odom;
}

void rcvGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
{  
   if(!_has_odom)
   {
        ROS_ERROR("[waypoint_generator] No odom!");
        return;
   }

   _pose = *msg;
   double yaw = tf::getYaw(_pose.pose.orientation);
   //cout<<"yaw angle: "<<yaw<<endl;

   randomObsGenerator(yaw, _pose.pose.position.x, _pose.pose.position.y, _pose.pose.position.z);
}

int main (int argc, char** argv) 
{        
   ros::init (argc, argv, "click_new_obstacle_generator");
   ros::NodeHandle nh( "~" );

   _obs_pub     = nh.advertise<sensor_msgs::PointCloud2>("click_new_obs", 1);                      
   _obs_all_pub = nh.advertise<sensor_msgs::PointCloud2>("click_new_obs_added", 1);                      
   _odom_sub    = nh.subscribe("odometry", 50, rcvOdometryCallback );
   _goal_sub    = nh.subscribe("goal",     10, rcvGoalCallback     );

   nh.param("map/resolution", _resolution, 0.2);
   nh.param("map/max_height", _max_z,      3.0);

   nh.param("ObstacleShape/lower_rad", _w_l,   0.3);
   nh.param("ObstacleShape/upper_rad", _w_h,   0.8);
   nh.param("ObstacleShape/lower_hei", _h_l,   2.0);
   nh.param("ObstacleShape/upper_hei", _h_h,   5.0);
   
   ros::Rate rate(100);
   bool status = ros::ok();
   while(status) 
   {
     ros::spinOnce();  
     status = ros::ok();
     rate.sleep();
   }
   return 0;
}