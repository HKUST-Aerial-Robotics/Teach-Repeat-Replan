roscore & sleep 1;

roslaunch djiros djiros.launch & sleep 5;

# your sensor suits VINS, such as:
roslaunch realsense2_camera rs_camera.launch & sleep 4;
rosrun dynamic_reconfigure dynparam set /camera/Stereo_Module 'Emitter Enabled' false & sleep 0.1;

rosrun vins vins_node ~/catkin_ws/src/VINS-Fusion/config/newcalibration/realsense_stereo_imu_config.yaml & sleep 1;
rosrun loop_fusion loop_fusion_node ~/catkin_ws/src/VINS-Fusion/config/newcalibration/realsense_stereo_imu_config.yaml & sleep 1;
