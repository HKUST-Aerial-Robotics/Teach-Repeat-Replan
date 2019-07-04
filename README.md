This is the experiment verision of the Teach-Repeat-Replan system. To run this system in real-world, please follow the **Tutorial** in this page.

*We will soon complete the Tutorial*

To test Teach-Repeat-Replan system in simulation, please go to the [master](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) branch.

# Teach-Repeat-Replan (Autonomous Drone Race)
Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments

**Teach-Repeat-Replan** is a complete and robust system enables **Autonomous Drone Race**. It contains all components for UAV aggressive flight in complex environments. It is built upon on the classical robotics teach-and-repeat framework, which is widely adopted in infrastructure inspection, aerial transportation, and search-and-rescue. Our system can capture users' intention of a flight mission, convert an arbitrarily jerky teaching trajectory to a guaranteed smooth and safe repeating trajectory, and generate safe local re-plans to avoid unmapped or moving obstacles on the flight.

<p align = "center">
<img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/drone_race_1.gif" width = "413" height = "264" border="5" />
<img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/drone_race_2.gif" width = "413" height = "264" border="5" />
</p>

**Architecture:**
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/sys_architecture.png" width = "767" height = "366">
  </p>

**Related Papers**

* [**Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments**](), Fei Gao, Luqi Wang, Boyu Zhou, Luxin Han, Jie Pan, shaojie Shen, submitted to IEEE Transactions on Robotics (**T-RO**).

* [**Optimal Trajectory Generation for Quadrotor Teach-and-Repeat**](https://ieeexplore.ieee.org/abstract/document/8625495), Fei Gao, Luqi Wang, Kaixuan Wang, William Wu, Boyu Zhou, Luxin Han, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.

*If you use Teach-Repeat-Replan or its sub-modules for your application or research, please cite our related papers.* [bib](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/bib.txt)

# Hardware

The onboard computer (carried on the drone) communicates with the ground station with Wifi or Ethernet. As shown in the below figure. Remember to start the roscore on the onboard computer, to reduce the delay of data transfer while controlling the drone.

 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/experiment/files/hardware.png" width = "520" height = "400">
  </p>

# Tutorial
## 1. Installation

To try the **Teach-Repeat-Replan** system in real-world experiments, the installation should be done in a ground station and an onboard computer, respectively.

**1.1**  **Ground Station**

Put the folder ```ground_station``` into the workspace of a ground station computer. Install all dependencies following the 
```install_tools```. Compile it by ```catkin_make```.

**1.2**  **Onboard Computer**

Put the folder ```onboard_computer``` into the workspace of the onboard computer of your drone. 

For **sensors**, in this project, we use [**realsense D435i**](https://www.intelrealsense.com/depth-camera-d435i/), which is a stereo pair with IMU. However, we use the IMU from DJI N3 autopilot since it's more stable. Therefore, you can also use [**realsense D435**](https://www.intelrealsense.com/depth-camera-d435/).

You should first install its SDK [librealsense](https://github.com/IntelRealSense/librealsense).

*We found that by the date 30/06/2019, the latest version of realsense driver has a bug, so we recommend to use a [older version](https://github.com/IntelRealSense/librealsense/releases) 19.1*

Then install its [ros-wrapper](https://github.com/IntelRealSense/realsense-ros).

For **local mapping** and **re-planning**, the local replanner depends on ```NLopt```, install in by 
```
  sudo apt-get install libnlopt-dev
```

For **localization**, install **VINS** follow the [instruction](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/localization/VINS-Fusion).

For **controller**, install **DJI_ROS** and **N3Ctrl** follow the [instruction1](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/djiros) and [instruction2](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/n3ctrl).

## 2. Usage

**2.1**  Mapping Phase

The mapping process builds a dense and globally consistent map, which works as a prior map for following the autonomous flight.
To start mapping, in the ```ground_station``` folder, run:

```
    ./mapping.sh
```

Then in the onboard computer, run sensors and localization by:

```
    ./sensing_estimation.sh
```

Then the mapping module starts building a dense map. When the program is shut down, the mapping result (a point cloud) will be saved locally under the directory of ```ground_station/global_mapping/surfel_fusion```.

After mapping, shut down both programs in the ground station and in the onboard computer.

**2.2**  Teaching Phase

For experiments, the teaching can be done as the same as in the simulation. Follow the instruction in [master](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) to teach the drone your expected path. The script to run the teaching is

```
    ./teaching_planning.sh
```
After the teaching, **Do Not** shut down this program, because global planning will be done later in the ground station.

**2.3**  Repeating Phase

After all the above procedures are done, run:
```
    ./ctrl_estimation_replanning.sh
```
in the onboard computer, to launch the controller, local mapping, and local re-planning modules. Then take off the drone and pull the trigger in the remote controller to start a repeating mission.
