rosrun joy joy_node & sleep 0.1;
roslaunch cascade_controller simulator.launch drone_number:=$i yaw:=true & sleep 1;
roslaunch trr_global_planner  map_generator.launch   & sleep 1;
roslaunch trr_global_planner  global_planner.launch  & sleep 1;
roslaunch trr_local_replanner local_replanner.launch & sleep 1;
wait
