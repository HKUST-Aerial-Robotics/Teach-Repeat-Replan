## A modified [dji-sdk/Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS) ##

This is a modified version of [dji-sdk/Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS), which uses standard ros message types, provides limited function of [dji-sdk/Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS).

### For Beginners, Please Follow [Step-by-Step Tutorial](docs/step_by_step_tutorial.md) ###

### TLDR for Experts ###

* FindEigen.cmake from **ceres-solver** is used. Please resolve it accordingly if you meet some problem about Eigen.

* Remember to install [dji-sdk/Onboard-SDK](https://github.com/dji-sdk/Onboard-SDK). Follow instructions of [DJI-Onboard-SDK Documentation](https://developer.dji.com/onboard-sdk/documentation/sample-doc/sample-setup.html#linux-oes) to install it into the system. 

* If you don't need mvBlueFOX synchronization, just [set ENABLE_DJIFOX to false](CMakeLists.txt#L22) to eliminate compile errors about mvBlueFOX drivers. If you need it, please install [the bluefox camera drivers](https://www.matrix-vision.com/USB2.0-single-board-camera-mvbluefox-mlc.html).

* **launch/djiros.launch** and **launch/djifox.launch** will use environment variables to get APPID and ENCKEY. You can add your id and key in your launch file / set it to the environment. For members of HKUST-Aerial-Robotics-Group, please see ```Dropbox/reading/code/uavteam/DJISDK_APP_KEY.txt``` for id and key.

* Please pay attention to ttyUSB name and privilege. An example for udev-rule file is provided in [docs/99-ftdi.rules](docs/99-ftdi.rules). A tutorial can be found at http://hintshop.ludvig.co.nz/show/persistent-names-usb-serial-devices/

### ROS Interfaces ###

#### Parameters ####
```
~serial_name             [string] : Path to the serial port device (e.g. /dev/ttyUSB0)
~baud_rate               [int]    : Baudrate for serial port
~app_id                  [int]    : App Id for dji sdk
~enc_key                 [string] : App Key for dji sdk
~sensor_mode             [bool]   : No activation and control is needed, just output imu, rc, gps, ...
~align_with_fmu          [bool]   : Use ticks from FMU/ ros::Time::now() when data is received.
~gravity                 [double] : scale multiplied on accelerometer output
~ctrl_cmd_stream_timeout [double] : timeout for judging if control command is streaming in or stopped
~ctrl_cmd_wait_timeout   [double] : timeout for waiting for control command after switch into api mode
```

#### Camera Parameters ####

Please see [KumarRobotics/bluefox2](https://github.com/KumarRobotics/bluefox2) as a reference

#### Topics ###

Publishers:

```
 ~imu  : [sensor_msgs/IMU]              IMU message at 400 Hz, in the [ROS REP 103](http://www.ros.org/reps/rep-0103.html) frame definition.
 ~rc   : [sensor_msgs/Joy]              RC joysticks at 50 Hz, remapped to [-1, +1]. See [include/djiros/DjiRos.h](include/djiros/DjiRos.h) for the direction.
 ~velo : [geometry_msgs/Vector3Stamped] Velocity message in NED frame.
 ~gps  : [sensor_msgs/NavSatFix]        GPS message.
 [SYNC] ~image: [sensor_msgs/Image]     Synchronized images from camera driver.
```

* See the code and [official documents](https://developer.dji.com/onboard-sdk/documentation/) for published topics and their details.
* For other messages, please modify the code.
* Pay attention to the frame_id of the published topics which represent the frame. Frame definition can be seen in [include/djiros/DjiRos.h](include/djiros/DjiRos.h).

Subscribers:
```
~ctrl: [sensor_msgs/Joy] For controlling the drone
```
<!-- * Subscriber "~gimbal_ctrl" and "~gimbal_speed_ctrl" for control the gimbal -->

<!-- #### TODO #### -->

$ djiros_bluefox
