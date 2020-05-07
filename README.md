This is the experiment verision of the Teach-Repeat-Replan system. To run this system in real-world, please follow the **Tutorial** in this page.

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

* [**Teach-Repeat-Replan: A Complete and Robust System for Aggressive Flight in Complex Environments**](), Fei Gao, Luqi Wang, Boyu Zhou, Luxin Han, Jie Pan, shaojie Shen, IEEE Transactions on Robotics (**T-RO**), to appear, 2020.


* [**Optimal Trajectory Generation for Quadrotor Teach-and-Repeat**](https://ieeexplore.ieee.org/abstract/document/8625495), Fei Gao, Luqi Wang, Kaixuan Wang, William Wu, Boyu Zhou, Luxin Han, Shaojie Shen, IEEE Robotics and Automation Letters (**RA-L**), 2019.

*If you use Teach-Repeat-Replan or its sub-modules for your application or research, please cite our related papers.* [bib](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/master/files/bib.txt)

# Hardware

The onboard computer (carried on the drone) communicates with the ground station with Wifi or Ethernet. As shown in the below figure. Remember to start the roscore on the onboard computer, to reduce the delay of data transfer while controlling the drone.

 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/experiment/files/hardware.png" width = "520" height = "400">
  </p>

# Tutorial

## 1. Tested Hardware/Software

The hardware/software combination that passed our test below are **dji N3** flight controller, **dji Manifold 2-C** onboard computer, **dji Lightbridge2** remote controller, **intel realsense D435i** camera and **Ubuntu 16.04**.

## 2. Install drivers and dependencies

- Run ```./install_tools.sh``` script in the src folder to install necessary dependencies. If anything goes wrong, please open this script and manually install them one by one.
- Follow the [instruction1](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/tree/experiment/onboard_computer/controller/djiros) to install the drivers for **N3** flight controller on onboard computer. [version 3.7 of Onboard-SDK](https://github.com/dji-sdk/Onboard-SDK/tree/3.7) is recomended.
- Follow the [instruction2](https://github.com/IntelRealSense/librealsense) to install the drivers for **intel realsense** on onboard computer.
- Install necessary solvers [Mosek](https://www.mosek.com/), [OOQP](http://pages.cs.wisc.edu/~swright/ooqp/), [Ceres](http://ceres-solver.org/) and [NLopt](https://nlopt.readthedocs.io/en/latest/). You can refer to [simulation](https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan) for more details.

## 3. Compile the code on onboard computer

Create a ros workspace on onboard computer and src folder in this directory, move all files in *onboard_computer* to newly created src folder, and compile the project in the workspace with ```catkin_make``` command. If you are prompted for a lack of libraries, Google them and install the missings. Finally, add the script ```source <your_catkin_ws>/devel/setup.bash``` to *~/.bashrc*.

## 4. Configure and run the VIO module -- vins-fusion

1. **Run djiros to get IMU data from N3**
<br/>Use the USB cable to connect the N3 flight controller to DJI Assistant2 software, check the **Enable API** box and select baud rate to **921600**, then unplug and plug the power cable to restart the flight controller.
<br/>Use **CP2102** module to connect Manifold-2C to N3 flight controller. Fill out the APP ID and APP KEY parameter to *&lt;your_catkin_ws&gt;/src/djictrl/djiros/launch/djiros.launch*. For more detials, please refer to [dji developer website](https://developer.dji.com/onboard-sdk/documentation/development-workflow/environment-setup.html) and N3 flight controller datasheet.
<br/>Run ```roslaunch djiros djiros.launch```, and then run ```rostopic hz /djiros/imu```. It can be seen that the frequency of data output of imu is 400HZ, which means success.

2. **Run realsense to get depth and grayscale images**
<br/>Just run ```roslaunch realsense2_camera rs_camera.launch```. If an error is reported, try replugging the USB port. After that, run ```rqt_image_viewer``` and select */camera/depth/image_rect_raw* or */camera/infra1/image_rect_raw* to check if the display is correct.

3. **Configure camera parameters**
<br/>Enter *&lt;your_catkin_ws&gt;/src/vio/vins-fusin/config/realsense/* folder, where lies the main configuration files for vins-fusin. Our hardware/software configuration matches *realsense_stereo_imu_config.yaml* where you must properly reconfigure the extrinsics of the camera and IMU (**body_T_cam0**, **body_T_cam1**, **center_T_imu**). 
<br/>Then in the same directory, open *left.yaml*, which is the parameters profile for the left camera. Configure **camera resolution**, **distortion factor**, **intrinsic parameters**, etc. Similarly, open the *right.yaml* file and fill in the parameters for the right camera.

4. **Run VINS-Fusion**
<br/>Now it is time to run the whole VIO system. Enter *&lt;your_catkin_ws&gt;/src/* directory, and then open *sensing_estimation.sh*, change the path to your own *realsense_stereo_imu_config.yaml* file at the last **two** lines and save the file.
Make sure that all the ros nodes are killed and run ```./sensing_estimation.sh``` in *&lt;your_catkin_ws&gt;/src/* directory. It will take about ten seconds to finish the initialization, and after that accurate and fast camera pose and trajectory can be seen in rviz as show in the **fig.1** that if all the parameters are configured properly. 

 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/experiment/files/vio.png" width = "800" height = "593">
  </p>
  <center>Fig.1</center>
  
## 5. Communication between Ground Computer and Onboard Computer

1. Connect the ground computer and the onboard computer into the same LAN network so that they can ping to each other. Enter the ```ifconfig``` command to get the IP addresses at both computers. If the IP of the onboard computer is 192.168.1.100 and the IP of the ground computer is 192.168.1.101, than at the end of the *~/.bashrc* on onboard computer, add the code below:

```shell
export ROS_HOSTNAME = 192.168.1.100
export ROS_MASTER_URI = http://192.168.1.100:11311
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;While on ground computer add the code below in the same file:

```shell
export ROS_HOSTNAME = 192.168.1.101
export ROS_MASTER_URI = http://192.168.1.100:11311
```
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Then ```source ~/.bashrc``` on both computers.

2. Communication Test: Running any node on the onboard computer and then entering ```rostopic list``` on the ground computer, topics should show up on the ground side, which means communication success.

## 6. Mapping

 - Compile the code in *ground_station* folder in ROS workspace on the ground computer.
 
 - Create a new folder *~/output/pose_graph/* on the onboard computer to store the pose graph generated during mapping. Run ```./sensing_estimation.sh```. (It is highly recommended that the rviz statement be manually commented out to save computing resources.)
 
 - Open *&lt;your_catkin_ws&gt;/src/global_mapping/surfel_fusion/launch/global_map_fusion.launch* on the ground computer and correctly modify the intrinsics and extrinsics of the camera.
 
 - Modify the path of the *.rviz* file in *mapping.sh*, and then run ```./mapping.sh```. Move the quadrotor around and you will see **Fig.2** below if everything goes well.
 
 <p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/experiment/files/mapping_visualization.png" width = "800" height = "477">
  </p>
  <center>Fig.2</center>
  
 - After the mapping phase is finished, the ground computer can be closed by **Ctrl+C**. The program will save the point cloud data automatically before exiting and the file name is *model.pcd*. The onboard computer can **not** use Ctrl+C to shut down directly after mapping phase for that it needs to save the pose graph got during mapping and this process must be operated by inputing "**s**" and **press "Enter"** manually. Then the onboard computer will save the recorded pose graph to the *~/output/pose_graph* folder.

**P.S.** 

 - Ensure that the network speed between the onboard and the ground
   computer can reach at least **20MB/s**. The communication speed can
   be viewed in the **system monitor** software. If the speed of
   wireless network is not enough, it is recommended to use wired
   connection during mapping.

 - If you want to rebuild the map, you need to delete all the existed files
   in the *~/output/pose_graph/* folder.

## 7. Test Whether the Quadrotor is Running Properly

1. Modify the path of the *realsense_stereo_imu_config.yaml* file related to **VINS** in the script file *ctrl_estimation_replanning.sh*, and then run the ```./ctrl_estimation_replanning.sh``` script on the onboard computer. 

2. Try to fly. Firstly, put the **small switch**(see the left picture of Fig.3) lies in the **left upper** side of the remote controller to stay at position **"A"**, and move the **"H"** switch(see the right picture of Fig.3) that in **right bottom** side of the remote controller to hit **up**. Then take the quadrotor off, in the air, move the **left upper switch** from **A to P** and the aircraft should perform fixed-point immediately that stablely hovering in the air, that means the controller operates correctly.

<p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/experiment/files/remote_controller.png" width = "800" height = "304">
  </p>
  <center>Fig.3</center>

## 8. Perform Teach-Repeat-Replan

1. Modify the camera parameters in *&lt;your_catkin_ws&gt;/src/local_replanner/local_replanner/launch/local_replanner.launch*, and then run ```ctrl_estimation_replanning.sh``` on the onboard computer, waiting for pose graph to be added (possibly more than 1 minute), which will be prompted in terminal after loading.

2. Modify the name of point cloud file during mapping in the *teaching_planning.sh* script file, and then run the ```./teaching_planning.sh```. The rviz will be like Fig.4.

<p align="center">
  <img src="https://github.com/HKUST-Aerial-Robotics/Teach-Repeat-Replan/blob/experiment/files/teaching_phase.png" width = "800" height = "612">
  </p>
  <center>Fig.4</center>

3. Then use the Xbox joystick or keyboard to draw a trajectory in rviz(teach phrase).

4. Start the quadrotor with a remote control while keeping the upper left switch be selected in **A** and the lower right "H" switch in the position **up**. At an altitude of about one meter, move the upper left switch from **A to P**, the quadrotor will enter a stable hovering state, and then move the lower right "H" switch **down**. If nothing goes wrong, the quadrotor will automatically start to execute the desired trajectory.

**So far, the whole system has run successfully.**

## 9. trouble shooting

1. If there is an inexplicable error shows up, you can try to restart the onboard computer and the ground computer.
2. Try **not** to run rviz on the onboard computer, which will take up too much resources and do negtive affect to the performance.
3. If the quadrotor can not be armed, then we need to unplug the USB/serial module and restart the flight controller, finally plug the USB/serial module. Make sure the USB/serial module is disconnected when the flight controller is powering on.
4. If realsense fails to start, it can be solved by unplug/plug again.
5. Always pay attention to the quality of communication, high latency and low network speed, which will cause unexpected problems.
6. If it is not possible to generate trajectories during teaching and repeat phase, it is recommended that teach slowly and ensure that all polyhedrons generated in the rviz are uninterrupted.
7. For the local replaner is unstable in realworld, it has been blocked in the code temporary. Please wait for updates. The lack of local replaner does not affect the global trajectory generation and flight, but can not avoid unexpected obstacles during flying.
