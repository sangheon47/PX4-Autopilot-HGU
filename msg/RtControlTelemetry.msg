uint8 ORB_QUEUE_LENGTH = 4

uint64 timestamp            # time since system start (microseconds)
uint32 cycle
float64 ideal_start_s
float64 actual_start_s
uint32 exec_us
uint32 input_us
uint32 control_us
uint32 output_us
uint32 slack_us
uint32 missed_cycles
float32[3] accel_m_s2
float32[3] gyro_rad_s
float32[6] motor_norm
uint16[4] servo_pwm_us
uint16[2] bldc_dshot
