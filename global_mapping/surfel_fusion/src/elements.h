#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

struct Superpixel_seed
{
    float x, y;
    float size;
    float norm_x, norm_y, norm_z;
    float posi_x, posi_y, posi_z;
    float view_cos;
    float mean_depth;
    float mean_intensity;
    bool fused;
    bool stable;

    // for debug
    float min_eigen_value;
    float max_eigen_value;
};

struct SurfelElement
{
    float px, py, pz;
    float nx, ny, nz;
    float size;
    float color;
    float weight;
    int update_times;
    int last_update;
};
