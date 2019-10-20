#roscore & sleep 1;
roslaunch djiros djiros.launch & sleep 5;

# your sensor suits VINS, such as:
roslaunch realsense2_camera rs_camera.launch & sleep 5;

rosrun vins vins_node /home/dji/dji/catkin_ws_basic/src/vio/VINS-Fusion/config/realsense/realsense_stereo_imu_config.yaml  & sleep 5;
rosrun loop_fusion loop_fusion_node /home/dji/dji/catkin_ws_basic/src/vio/VINS-Fusion/config/realsense/realsense_stereo_imu_config.yaml & sleep 1;

rviz -d /home/dji/dji/catkin_ws_basic/src/vio/VINS-Fusion/config/vins_rviz_config.rviz
