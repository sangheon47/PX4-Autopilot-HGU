#include "rt_control.h"

#include <assert.h>
#include <string.h>

#if (RT_CTRL_TX_Q_LEN == 0U) || ((RT_CTRL_TX_Q_LEN & (RT_CTRL_TX_Q_LEN - 1U)) != 0U)
#error "RT_CTRL_TX_Q_LEN must be a non-zero power of two"
#endif

static uint16_t queue_next(uint16_t idx)
{
	return (uint16_t)((idx + 1U) & (RT_CTRL_TX_Q_LEN - 1U));
}

void rt_control_state_reset(rt_control_state_t *state)
{
	assert(state != NULL);

	memset(state, 0, sizeof(*state));
}

void rt_control_queue_reset(rt_control_queue_t *queue)
{
	assert(queue != NULL);

	memset(queue, 0, sizeof(*queue));
}

void rt_controller(const float accel_m_s2[3],
		   const float gyro_rad_s[3],
		   const rt_control_opti_sample_t *opti_sample,
		   float motor_norm_out[RT_CTRL_MOTOR_COUNT])
{
	assert(accel_m_s2 != NULL);
	assert(gyro_rad_s != NULL);
	assert(motor_norm_out != NULL);
	const bool opti_available = (opti_sample != NULL) && opti_sample->valid;

	// Body-frame accelerometer axes [m/s^2]
	// X: accel_m_s2[0], 		Y: accel_m_s2[1], 		Z: accel_m_s2[2]

	// Body-frame gyroscope axes [rad/s]
	// X(roll): gyro_rad_s[0], 	Y(pitch): gyro_rad_s[1], 	Z(yaw): gyro_rad_s[2]

	// External OptiTrack sample [m, rad] when available:
	// opti_sample->x, opti_sample->y, opti_sample->z
	// opti_sample->roll, opti_sample->pitch, opti_sample->yaw
	// opti_sample->seq, opti_sample->age_us
	if (opti_available) {
		// opti_sample->x, y, z
		// opti_sample->roll, pitch, yaw
		// opti_sample->seq, age_us
	} else {
		// IMU-only fallback
	}

	// Normalized actuator commands [0..1]:
	// Servo1..Servo4 and BLDC1..BLDC2.
	const float servo1_norm = 0.5f;
	const float servo2_norm = 0.5f;
	const float servo3_norm = 0.5f;
	const float servo4_norm = 0.5f;
	const float bldc1_norm = 0.1f;
	const float bldc2_norm = 0.1f;

	motor_norm_out[0] = servo1_norm; // Servo1
	motor_norm_out[1] = servo2_norm; // Servo2
	motor_norm_out[2] = servo3_norm; // Servo3
	motor_norm_out[3] = servo4_norm; // Servo4
	motor_norm_out[4] = bldc1_norm;  // BLDC1
	motor_norm_out[5] = bldc2_norm;  // BLDC2
}

void rt_control_queue_enqueue(rt_control_queue_t *queue, const rt_control_telemetry_frame_t *frame)
{
	assert(queue != NULL);
	assert(frame != NULL);

	const uint16_t head = __atomic_load_n(&queue->head, __ATOMIC_RELAXED);
	const uint16_t tail = __atomic_load_n(&queue->tail, __ATOMIC_ACQUIRE);
	const uint16_t next = queue_next(head);

	if (next == tail) {
		__atomic_add_fetch(&queue->drop_count, 1U, __ATOMIC_RELAXED);
		return;
	}

	queue->q[head] = *frame;
	__atomic_store_n(&queue->head, next, __ATOMIC_RELEASE);
}

bool rt_control_queue_copy_peek(const rt_control_queue_t *queue, rt_control_telemetry_frame_t *out_frame)
{
	assert(queue != NULL);
	assert(out_frame != NULL);

	const uint16_t tail = __atomic_load_n(&queue->tail, __ATOMIC_RELAXED);
	const uint16_t head = __atomic_load_n(&queue->head, __ATOMIC_ACQUIRE);

	if (head == tail) {
		return false;
	}

	*out_frame = queue->q[tail];
	return true;
}

void rt_control_queue_pop(rt_control_queue_t *queue)
{
	assert(queue != NULL);

	const uint16_t tail = __atomic_load_n(&queue->tail, __ATOMIC_RELAXED);
	const uint16_t head = __atomic_load_n(&queue->head, __ATOMIC_ACQUIRE);

	if (head == tail) {
		return;
	}

	__atomic_store_n(&queue->tail, queue_next(tail), __ATOMIC_RELEASE);
}
