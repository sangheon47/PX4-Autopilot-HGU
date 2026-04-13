uint8 ORB_QUEUE_LENGTH = 4

uint64 timestamp                # [us] control-cycle start time since boot
uint32 loop_dt_us               # [us] actual interval since the previous control cycle
uint32 exec_us                  # [us] total realtime-loop execution time
float32[3] accel_m_s2           # [m/s^2] latest control-loop acceleration
float32[3] gyro_rad_s           # [rad/s] latest control-loop angular rate
uint16[4] motor_pwm_us          # [us] final PWM sent to the 4 motor ESC outputs
float32[3] vision_pos_m         # [m] latest visual odometry position, NaN if invalid
float32[3] vision_rpy_rad       # [rad] latest visual odometry roll/pitch/yaw, NaN if invalid
uint32 vision_age_us            # [us] age of the latest visual odometry sample
uint8 vision_valid              # 1 when the visual odometry sample is finite and fresh
