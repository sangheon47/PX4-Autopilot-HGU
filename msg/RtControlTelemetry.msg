uint8 ORB_QUEUE_LENGTH = 4

uint64 timestamp            # time since system start (microseconds)
uint32 cycle
float64 actual_start_s
uint32 exec_us
uint32 input_us
uint32 control_us
uint32 output_us
uint32 missed_cycles
float32[3] accel_m_s2
float32[3] gyro_rad_s
float32[6] motor_norm
float32 opti_x
float32 opti_y
float32 opti_z
float32 opti_roll
float32 opti_pitch
float32 opti_yaw
uint32 opti_seq
uint32 opti_age_us
uint8 opti_valid
