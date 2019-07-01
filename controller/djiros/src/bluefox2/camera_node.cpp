#include <ros/ros.h>

#include "camera.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bluefox2");
    bluefox2::Camera camera(ros::NodeHandle("~"));

    if (camera.isOK())
        camera.feedImages();
    return 0;
}
