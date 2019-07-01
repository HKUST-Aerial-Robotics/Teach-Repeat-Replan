# VINS-Fusion
## An optimization-based multi-sensor state estimator
**In our experiment, we use images from realsense D435i and Imu from N3 controller to run VINS-Fusion.** 

This folder is a customized VINS-Fusion, with some code and logic changed for our Teach-Repeat-Replan system. In the mapping phase, the map is based on a global frame determined by the pose graph optimization. While during the Repeat-Replan phase, the current VIO frame may drift a lot from the global frame. We publish relative poses between these two frames when controlling the drone, to compensate for the pose drifts.

**For technical details, please check [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**  

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)


### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).


## 2. Build VINS-Fusion
Clone the repository and catkin_make:
```
 cd ~/catkin_ws/src
 git clone https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git
 cd ../
 catkin_make
 source ~/catkin_ws/devel/setup.bash
```
(if you fail in this step, try to find another computer with clean system or reinstall Ubuntu and ROS)

## 3. Topics 
Subscribers:  
```
 /djiros/imu:  [sensor_msgs/Imu]                        imu messages for VINS-Fusion getting from djiros 
 /camera/infra1/image_rect_raw: [sensor_msgs/Image]     left image for VINS-Fusion getting from realsense D435i 
 /camera/infra2/image_rect_raw: [sensor_msgs/Image]     right image for VINS-Fusion getting from realsense D435i 
``` 

Publishers:
```
 /vins_estimator/camera_pose:   [geometry_msgs/PoseStamped]  left camera pose, used for building local and global map
 /vins_estimator/imu_propagate: [nav_msgs/Odometry]          pose of the drone at high frequence, used for control the drone
 /loop_fusion/pg_T_vio:         [geometry_msgs/Pose]         relative pose between current vio frame and global map frame
```
