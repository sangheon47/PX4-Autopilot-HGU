# 7-Nano Telemetry

## Purpose

`rt_control_telemetry` is intentionally compact.
It only carries data that is directly useful for realtime control experiments and post-run analysis.

Current field set:

- `timestamp` `[us]`: control-cycle start time since boot
- `loop_dt_us` `[us]`: measured interval from the previous control cycle
- `exec_us` `[us]`: measured realtime-loop execution time
- `accel_m_s2[3]` `[m/s^2]`: latest IMU acceleration used by the control loop
- `gyro_rad_s[3]` `[rad/s]`: latest IMU angular rate used by the control loop
- `motor_pwm_us[4]` `[us]`: final PWM sent to the 4 motor outputs
- `vision_pos_m[3]` `[m]`: latest visual odometry position, `NaN` if invalid
- `vision_rpy_rad[3]` `[rad]`: latest visual odometry roll/pitch/yaw, `NaN` if invalid
- `vision_age_us` `[us]`: age of the latest visual odometry sample
- `vision_valid` `[0/1]`: whether the visual odometry sample is finite and fresh

Intentionally removed from telemetry:

- `SAMPLE_PERIOD`
- `SAMPLE_DT`
- cycle counter
- input/control/output timing split
- normalized motor percent output
- old `opti_*` duplicates

Reason:

- `loop_dt_us` already gives the real measured cycle interval.
- `motor_pwm_us` is the actual actuator command that matters for experiments.
- keeping the payload small makes MAVLink tunnel logging simpler and more reliable.

## CLI Example

`iim42652 status` style output:

```text
RT sample_time_s=0.005000 freq_hz=200.0 telem_queue=2/512 dropped=0 high_water=6
RT motor_pwm initialized=true rate_hz=200 mask=0xf output_mode=auto test_pwm_us=1000
RT time loop_dt_us=5000 exec_us=142
RT accel_m_s2: x=0.02100 y=-0.01520 z=9.80110 gyro_rad_s: roll=0.00310 pitch=-0.00180 yaw=0.00040
RT motor_pwm_us=1000 1000 1000 1000
RT vision age_us=18345 pos_m=(1.2034, -0.1050, 0.4820) rpy_rad=(0.0120, -0.0210, 1.5700)
RT telem topic=rt_control_telemetry advertised=true pub_ok=4120 pub_fail=0
```

If visual odometry is stale or invalid:

```text
RT vision invalid age_us=250000
```

## CSV Logger Example

`Tools/telem_csv_logger.py` writes this header:

```text
timestamp_us,loop_dt_us,exec_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,pwm_1,pwm_2,pwm_3,pwm_4,vision_x,vision_y,vision_z,vision_roll,vision_pitch,vision_yaw,vision_age_us,vision_valid
```

Example row:

```text
53218000,5000,142,0.0210,-0.0152,9.8011,0.0031,-0.0018,0.0004,1000,1000,1000,1000,1.2034,-0.1050,0.4820,0.0120,-0.0210,1.5700,18345,1
```

## MAVLink TUNNEL Payload

`RT_CONTROL_TELEMETRY` packs the same data into an 80-byte payload in this order:

```text
timestamp_us
loop_dt_us
exec_us
accel_m_s2[3]
gyro_rad_s[3]
motor_pwm_us[4]
vision_pos_m[3]
vision_rpy_rad[3]
vision_age_us
vision_valid
```

## Notes For Future Timed Tests

If later experiments need "run this command for N cycles" or "hold this thrust for T seconds":

- keep that counter or timer inside control state, not in telemetry by default
- prefer measured elapsed time or an internal cycle counter over a published telemetry field
- only publish the counter if the experiment itself needs it for analysis
