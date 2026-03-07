#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RT_CTRL_SERVO_COUNT 4U
#define RT_CTRL_BLDC_COUNT 2U
#define RT_CTRL_MOTOR_COUNT (RT_CTRL_SERVO_COUNT + RT_CTRL_BLDC_COUNT)
#define RT_CTRL_TX_Q_LEN 128U

typedef struct {
	uint32_t cycle;
	double ideal_start_s;
	double actual_start_s;
	uint32_t exec_us;
	uint32_t input_us;
	uint32_t control_us;
	uint32_t output_us;
	uint32_t slack_us;
	uint32_t missed_cycles;
	float accel_m_s2[3];
	float gyro_rad_s[3];
	float motor_norm[RT_CTRL_MOTOR_COUNT];
	uint16_t servo_pwm_us[RT_CTRL_SERVO_COUNT];
	uint16_t bldc_dshot[RT_CTRL_BLDC_COUNT];
} rt_control_telemetry_frame_t;

typedef struct {
	uint32_t cycle;
	uint64_t t0_us;
	uint64_t last_control_ts_us;
	uint32_t deadline_miss_count;
	uint32_t max_loop_dt_us;
} rt_control_state_t;

typedef struct {
	rt_control_telemetry_frame_t q[RT_CTRL_TX_Q_LEN];
	uint16_t head;
	uint16_t tail;
	uint32_t drop_count;
} rt_control_queue_t;

void rt_control_state_reset(rt_control_state_t *state);
void rt_control_queue_reset(rt_control_queue_t *queue);

void rt_controller(const float accel_m_s2[3],
		   const float gyro_rad_s[3],
		   float motor_norm_out[RT_CTRL_MOTOR_COUNT]);

void rt_control_queue_enqueue(rt_control_queue_t *queue, const rt_control_telemetry_frame_t *frame);
bool rt_control_queue_copy_peek(const rt_control_queue_t *queue, rt_control_telemetry_frame_t *out_frame);
void rt_control_queue_pop(rt_control_queue_t *queue);

#ifdef __cplusplus
}
#endif
