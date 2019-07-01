float64 latitude          # radian
float64 longitude         # radian
float32 altitude          # relative altitude from takeoff point
float32 damping_distance  # Bend length (effective coordinated turn mode only)
int16 target_yaw          # Yaw (degree)
int16 target_gimbal_pitch # Gimbal pitch
uint8 turn_mode           # 0: clockwise, 1: counter-clockwise
uint8 has_action          # 0: no, 1: yes
uint16 action_time_limit
MissionWaypointAction waypoint_action
