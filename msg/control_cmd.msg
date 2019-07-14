Header header
  uint32 seq
  time stamp
  string frame_id
float64 control_flag  # control_flag=0, back to initial position; control_flag=1, admittance control
float64[7] joint_speed_cmd
float64 joint0_jerk
float64 joint0_external_tau
float64 damping
