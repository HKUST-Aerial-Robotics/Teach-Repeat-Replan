# N3Ctrl #
## A high level controller compatible with DJI N3 flight controller

## 1. Prerequisites
1.1 **Ubuntu** and **ROS**
Ubuntu  16.04.
ROS Kinetic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

1.2 **Eigen**
Download Eigen and install (http://eigen.tuxfamily.org/index.php)
The code is tested in Eigen 3.3

1.3 **djiros**
Follow the instruction to install the specified version of djiros ()

1.4 **quadrotor_msgs**
Follow the instruction to install the specified version of quadrotor_msgs ()

## 2. Build N3Ctrl on ROS
Clone the repository and catkin_make:
```
    cd ~/catkin_ws/src
    git clone https://github.com/HKUST-Aerial-Robotics/N3Ctrl.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Control parameters
There is a sample config file ctrl_param_fpv.yaml in the config folder.
Change the parameters according the real drone. Tune the PID parameters of the hover state and tracking state accordingly.

The input roll and pitch angle and thrust to the N3 flight controller is directly related to Kp * e_p + Kv * e_v + Kv_i * &int; e_v + Ka * des_a.
If the input to the N3 flight controller is desired yaw rate, which means using yaw rate control, the input is equal to Kyaw * e_yaw. Otherwise, the desired yaw is directly sent to N3 flight controller.   

## 4. Topics
Subscribers:
```
 ~imu  : [sensor_msgs/IMU]               IMU message from djiros.
 ~joy   : [sensor_msgs/Joy]              RC message form djiros
 ~cmd : [quadrotor_msgs/PositionCommand] Position command message from the planner.
```
Publishers:
```
 ~traj_start_trigger  : [geometry_msgs/PoseStamped]        A trigger message is sent when enter command mode.
 ~desire_pose : [geometry_msgs/PoseStamped]     The desired pose of the controller.
 ~ctrl : [sensor_msgs/Joy] The output of the control signal. The axes are the roll, pitch, thrust, yaw or yaw rate, thrust mode (if thrust mode > 0, thrust = 0~100%; if mode < 0, thrust = -? m/s ~ +? m/s), yaw mode (if yaw_mode > 0, axes[3] = yaw; if yaw_mode < 0, axes[3] = yaw_rate) respectively. The roll, pitch and yaw or yaw rate are in FRD frame. 
```

There is a sample launch file ctrl_md.launch in the launch folder.

## 5. Finite state machine (FSM)
The detailed graph of the FSM is shown in **docs/fsm.pdf**