rosrun joy joy_node & sleep 1;

roslaunch joy_ctrl joy_ctrl.launch & sleep 1;

rosrun pcl_ros pcd_to_pointcloud global_mapping/surfel_fusion/model_balcony.pcd 1.0 _frame_id:=map  & sleep 1;

roslaunch trr_global_planner global_planner_exp.launch
# exec bash;
