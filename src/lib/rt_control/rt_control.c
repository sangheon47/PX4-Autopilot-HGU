#include "rt_control.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#if (RT_CTRL_TX_Q_LEN == 0U) || ((RT_CTRL_TX_Q_LEN & (RT_CTRL_TX_Q_LEN - 1U)) != 0U)
#error "RT_CTRL_TX_Q_LEN must be a non-zero power of two"
#endif

#define RT_CTRL_LOOP_DT_S 0.005f
#define RT_CTRL_COMP_ALPHA 0.98f
#define RT_CTRL_PI_F 3.14159265358979323846f

typedef struct {
	float tilt_y_rad;
	float tilt_z_rad;
	float tilt_y_ref_rad;
	float tilt_z_ref_rad;
	bool initialized;
	bool tilt_y_ref_initialized;
	bool tilt_z_ref_initialized;
} rt_tilt_state_t;

static rt_tilt_state_t g_tilt_state;
static volatile bool g_zero_reference_pending;

static float rt_wrap_pi(float angle_rad)
{
	while (angle_rad > RT_CTRL_PI_F) {
		angle_rad -= 2.0f * RT_CTRL_PI_F;
	}

	while (angle_rad < -RT_CTRL_PI_F) {
		angle_rad += 2.0f * RT_CTRL_PI_F;
	}

	return angle_rad;
}

static float rt_rad_to_deg(float angle_rad)
{
	return angle_rad * (180.0f / RT_CTRL_PI_F);
}

static float rt_update_complementary_angle(float predicted_angle_rad, float measured_angle_rad)
{
	const float innovation_rad = rt_wrap_pi(measured_angle_rad - predicted_angle_rad);
	return rt_wrap_pi(predicted_angle_rad + ((1.0f - RT_CTRL_COMP_ALPHA) * innovation_rad));
}

static bool rt_compute_accel_tilt(const float accel_m_s2[3], float *tilt_y_rad, float *tilt_z_rad)
{
	assert(accel_m_s2 != NULL);
	assert(tilt_y_rad != NULL);
	assert(tilt_z_rad != NULL);

	const bool accel_valid = isfinite(accel_m_s2[0]) && isfinite(accel_m_s2[1]) && isfinite(accel_m_s2[2])
				 && ((accel_m_s2[0] * accel_m_s2[0]) + (accel_m_s2[1] * accel_m_s2[1])
				     + (accel_m_s2[2] * accel_m_s2[2]) > 1e-6f);

	if (!accel_valid) {
		return false;
	}

	// Rocket-centric tilt estimation:
	// X is the longitudinal axis, so Y/Z rotations are the controllable tilt axes.
	*tilt_y_rad = atan2f(accel_m_s2[2], -accel_m_s2[0]);
	*tilt_z_rad = atan2f(accel_m_s2[1], -accel_m_s2[0]);
	return true;
}

static void rt_estimate_tilt(const float accel_m_s2[3], const float gyro_rad_s[3], rt_tilt_state_t *state)
{
	assert(accel_m_s2 != NULL);
	assert(gyro_rad_s != NULL);
	assert(state != NULL);

	float accel_tilt_y_rad = state->tilt_y_rad;
	float accel_tilt_z_rad = state->tilt_z_rad;
	const bool accel_valid = rt_compute_accel_tilt(accel_m_s2, &accel_tilt_y_rad, &accel_tilt_z_rad);

	if (!state->initialized) {
		state->tilt_y_rad = accel_valid ? accel_tilt_y_rad : 0.0f;
		state->tilt_z_rad = accel_valid ? accel_tilt_z_rad : 0.0f;
		state->initialized = true;
		return;
	}

	const float predicted_tilt_y_rad = state->tilt_y_rad + (gyro_rad_s[1] * RT_CTRL_LOOP_DT_S);
	const float predicted_tilt_z_rad = state->tilt_z_rad + (gyro_rad_s[2] * RT_CTRL_LOOP_DT_S);

	if (accel_valid) {
		state->tilt_y_rad = rt_update_complementary_angle(predicted_tilt_y_rad, accel_tilt_y_rad);
		state->tilt_z_rad = rt_update_complementary_angle(predicted_tilt_z_rad, accel_tilt_z_rad);

	} else {
		state->tilt_y_rad = rt_wrap_pi(predicted_tilt_y_rad);
		state->tilt_z_rad = rt_wrap_pi(predicted_tilt_z_rad);
	}
}

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

void rt_controller_reset(void)
{
	memset(&g_tilt_state, 0, sizeof(g_tilt_state));
	g_zero_reference_pending = false;
}

void rt_controller_zero_reference(void)
{
	g_zero_reference_pending = true;
}

void rt_controller_get_debug_state(rt_controller_debug_state_t *out_state)
{
	assert(out_state != NULL);

	memset(out_state, 0, sizeof(*out_state));

	out_state->tilt_y_deg = rt_rad_to_deg(g_tilt_state.tilt_y_rad);
	out_state->tilt_z_deg = rt_rad_to_deg(g_tilt_state.tilt_z_rad);
	out_state->tilt_y_ref_deg = rt_rad_to_deg(g_tilt_state.tilt_y_ref_rad);
	out_state->tilt_z_ref_deg = rt_rad_to_deg(g_tilt_state.tilt_z_ref_rad);
	out_state->tilt_y_rel_deg = rt_rad_to_deg(rt_wrap_pi(g_tilt_state.tilt_y_rad - g_tilt_state.tilt_y_ref_rad));
	out_state->tilt_z_rel_deg = rt_rad_to_deg(rt_wrap_pi(g_tilt_state.tilt_z_rad - g_tilt_state.tilt_z_ref_rad));
	out_state->initialized = g_tilt_state.initialized ? 1U : 0U;
	out_state->tilt_y_ref_initialized = g_tilt_state.tilt_y_ref_initialized ? 1U : 0U;
	out_state->tilt_z_ref_initialized = g_tilt_state.tilt_z_ref_initialized ? 1U : 0U;
}

void rt_controller(const float accel_m_s2[3],
		   const float gyro_rad_s[3],
		   const rt_control_opti_sample_t *opti_sample,
		   float motor_norm_out[RT_CTRL_MOTOR_COUNT])
{
	assert(accel_m_s2 != NULL);
	assert(gyro_rad_s != NULL);
	assert(motor_norm_out != NULL);
	(void)opti_sample;

	rt_estimate_tilt(accel_m_s2, gyro_rad_s, &g_tilt_state);

	if (g_zero_reference_pending) {
		float zero_tilt_y_rad = g_tilt_state.tilt_y_rad;
		float zero_tilt_z_rad = g_tilt_state.tilt_z_rad;

		if (rt_compute_accel_tilt(accel_m_s2, &zero_tilt_y_rad, &zero_tilt_z_rad)) {
			g_tilt_state.tilt_y_rad = zero_tilt_y_rad;
			g_tilt_state.tilt_z_rad = zero_tilt_z_rad;
		}

		g_tilt_state.tilt_y_ref_rad = g_tilt_state.tilt_y_rad;
		g_tilt_state.tilt_z_ref_rad = g_tilt_state.tilt_z_rad;
		g_tilt_state.initialized = true;
		g_tilt_state.tilt_y_ref_initialized = true;
		g_tilt_state.tilt_z_ref_initialized = true;
		g_zero_reference_pending = false;
	}

	if (!g_tilt_state.tilt_y_ref_initialized) {
		g_tilt_state.tilt_y_ref_rad = g_tilt_state.tilt_y_rad;
		g_tilt_state.tilt_y_ref_initialized = true;
	}

	if (!g_tilt_state.tilt_z_ref_initialized) {
		g_tilt_state.tilt_z_ref_rad = g_tilt_state.tilt_z_rad;
		g_tilt_state.tilt_z_ref_initialized = true;
	}

	// The previous airframe used servo-actuated control. With four BLDC motors and no
	// quad/airframe-specific mixer defined yet, keep automatic outputs at minimum.
	for (uint8_t i = 0; i < RT_CTRL_MOTOR_COUNT; i++) {
		motor_norm_out[i] = 0.0f;
	}
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
