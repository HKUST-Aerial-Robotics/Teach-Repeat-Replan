#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub_map;
ros::Subscriber sub_map, sub_pose;
using namespace std;

void posCallBack(const geometry_msgs::PoseStamped pose)
{

}

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
ros::Time map_time;
void mapCallBack(const sensor_msgs::PointCloud2 & pointcloud_map)
{   
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pointcloud_map, *raw_cloud);
	map_time = pointcloud_map.header.stamp;
	
    input_cloud.reset( new pcl::PointCloud<pcl::PointXYZ> );
    if(raw_cloud->points.size() == 0)
        return;
    
    for(int i = 0; i < raw_cloud->points.size(); i++)
    {
        input_cloud->points.push_back(raw_cloud->points[i]);
    }   
}

void save_cloud(string save_path_name)
{
    printf("saving pointcloud ...\n");
    pcl::io::savePCDFile(save_path_name.c_str(), *cloud_filtered);
    printf("saving pointcloud done!\n");
}

void process( const ros::TimerEvent& event )
{	
	if(input_cloud->points.size() == 0) return;
	cloud_filtered.reset( new pcl::PointCloud<pcl::PointXYZ>);
    cloud_filtered->points.clear();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK (30);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    sensor_msgs::PointCloud2 filter_map;
    pcl::toROSMsg(*cloud_filtered, filter_map);
    filter_map.header.stamp = map_time;
    filter_map.header.frame_id = "/map";
    pub_map.publish(filter_map);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_server");
    ros::NodeHandle nh("~");

    sub_map  = nh.subscribe("/surfel_fusion/pointcloud",  1, mapCallBack);
    sub_pose = nh.subscribe("pose", 5000, posCallBack);

    pub_map  = nh.advertise<sensor_msgs::PointCloud2>("filtered_map", 1);

	ros::Timer  map_filter_timer = nh.createTimer(ros::Duration(2.0), process);

    string save_name;
    nh.param("save_name", save_name, string("error_log"));
    
    ros::Rate r(10);
    while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    cout<<"save_name: = "<<save_name<<endl;
    cout<<"save cloud"<<endl;
    string pcd_name = save_name + ".pcd";
    save_cloud(pcd_name);

    return 0;
}