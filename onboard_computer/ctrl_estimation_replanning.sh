#roscore & sleep 1;
roslaunch djiros djiros.launch & sleep 5;

roslaunch trr_local_replanner local_replanner.launch & sleep 1; #zxzx

# your sensor suits VINS, such as:
roslaunch realsense2_camera rs_camera.launch & sleep 4;

rosrun vins vins_node /home/dji/dji/catkin_ws/src/vio/VINS-Fusion/config/realsense/realsense_stereo_imu_config.yaml & sleep 1;
rosrun loop_fusion loop_fusion_node /home/dji/dji/catkin_ws/src/vio/VINS-Fusion/config/realsense/realsense_stereo_imu_config.yaml & sleep 5;

roslaunch n3ctrl ctrl_md.launch
