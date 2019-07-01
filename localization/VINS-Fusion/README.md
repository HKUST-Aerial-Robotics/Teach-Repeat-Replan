# VINS-Fusion
## An optimization-based multi-sensor state estimator
**In our experiment, we use images from realsense D435i and Imu from N3 controller to run VINS-Fusion.** 

**This folder is modified from VINS-Fusion, we change some code for our Teach-Repeat-Replan system. In the mapping phase, the map is based on that VIO frame which is regard as the global frame. While during the Repeat-Replan phase, the current VIO frame may be different from the global frame. So we publish the relative pose between these two coordinate systems to control the drone.**

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

## 4. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and [DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, a generic [camera model](https://github.com/hengli/camodocal) and [GeographicLib](https://geographiclib.sourceforge.io/).

## 5. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

We are still working on improving the code reliability. For any technical issues, please contact Tong Qin <qintonguavATgmail.com>.

For commercial inquiries, please contact Shaojie Shen <eeshaojieATust.hk>.
