/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "IIM42652.hpp"

#include <matrix/matrix/math.hpp>

#include <math.h>

void IIM42652::PublishSampleOutsideIRQ()
{
	LatestSample sample{};
	uint32_t seq_begin{0};
	uint32_t seq_end{0};

	do {
		seq_begin = _latest_publish_seq.load();

		if (seq_begin == 0 || (seq_begin & 1u)) {
			return;
		}

		sample = _latest_sample;
		seq_end = _latest_publish_seq.load();

	} while ((seq_begin != seq_end) || (seq_end & 1u));

	if (seq_end == _published_seq) {
		return;
	}

	_published_seq = seq_end;

	if (PX4_ISFINITE(sample.temperature)) {
		_px4_accel.set_temperature(sample.temperature);
		_px4_gyro.set_temperature(sample.temperature);
	}

	const uint32_t error_count = perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
				     + perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf);
	_px4_accel.set_error_count(error_count);
	_px4_gyro.set_error_count(error_count);

	_px4_accel.update(sample.timestamp_sample, sample.accel_raw[0], sample.accel_raw[1], sample.accel_raw[2]);
	_px4_gyro.update(sample.timestamp_sample, sample.gyro_raw[0], sample.gyro_raw[1], sample.gyro_raw[2]);
}

void IIM42652::UpdateLatestVisionPoseOutsideIRQ()
{
	vehicle_odometry_s odom{};

	if (!_vehicle_visual_odometry_sub.update(&odom)) {
		return;
	}

	LatestVisionPose sample{};
	sample.timestamp_sample = (odom.timestamp_sample != 0) ? odom.timestamp_sample : odom.timestamp;

	const bool position_valid = PX4_ISFINITE(odom.position[0])
				    && PX4_ISFINITE(odom.position[1])
				    && PX4_ISFINITE(odom.position[2]);
	const matrix::Quatf attitude_q(odom.q);
	const bool attitude_valid = attitude_q.isAllFinite();

	if (position_valid && attitude_valid) {
		const matrix::Eulerf attitude_euler(attitude_q);
		sample.position_m[0] = odom.position[0];
		sample.position_m[1] = odom.position[1];
		sample.position_m[2] = odom.position[2];
		sample.rpy_rad[0] = attitude_euler.phi();
		sample.rpy_rad[1] = attitude_euler.theta();
		sample.rpy_rad[2] = attitude_euler.psi();
		sample.valid = PX4_ISFINITE(sample.rpy_rad[0])
			       && PX4_ISFINITE(sample.rpy_rad[1])
			       && PX4_ISFINITE(sample.rpy_rad[2]);

	} else {
		sample.position_m[0] = NAN;
		sample.position_m[1] = NAN;
		sample.position_m[2] = NAN;
		sample.rpy_rad[0] = NAN;
		sample.rpy_rad[1] = NAN;
		sample.rpy_rad[2] = NAN;
		sample.valid = false;
	}

	const uint32_t seq = _latest_vision_pose_seq.load();
	_latest_vision_pose_seq.store(seq + 1);
	_latest_vision_pose = sample;
	_latest_vision_pose_seq.store(seq + 2);
}

bool IIM42652::CopyLatestVisionPose(LatestVisionPose &sample) const
{
	uint32_t seq_begin{0};
	uint32_t seq_end{0};

	do {
		seq_begin = _latest_vision_pose_seq.load();

		if (seq_begin == 0 || (seq_begin & 1u)) {
			return false;
		}

		sample = _latest_vision_pose;
		seq_end = _latest_vision_pose_seq.load();

	} while ((seq_begin != seq_end) || (seq_end & 1u));

	return true;
}

vision_pose_sample_t IIM42652::BuildVisionPoseSample(const hrt_abstime &cycle_begin,
		const LatestVisionPose &sample) const
{
	vision_pose_sample_t vision_pose{};
	vision_pose.position_m[0] = NAN;
	vision_pose.position_m[1] = NAN;
	vision_pose.position_m[2] = NAN;
	vision_pose.rpy_rad[0] = NAN;
	vision_pose.rpy_rad[1] = NAN;
	vision_pose.rpy_rad[2] = NAN;

	if (sample.timestamp_sample == 0U) {
		return vision_pose;
	}

	vision_pose.position_m[0] = sample.position_m[0];
	vision_pose.position_m[1] = sample.position_m[1];
	vision_pose.position_m[2] = sample.position_m[2];
	vision_pose.rpy_rad[0] = sample.rpy_rad[0];
	vision_pose.rpy_rad[1] = sample.rpy_rad[1];
	vision_pose.rpy_rad[2] = sample.rpy_rad[2];

	const uint64_t now_us = static_cast<uint64_t>(cycle_begin);
	const uint64_t age = (sample.timestamp_sample > 0 && now_us >= sample.timestamp_sample)
			     ? (now_us - sample.timestamp_sample)
			     : 0ULL;
	vision_pose.age_us = static_cast<uint32_t>(math::min<uint64_t>(age, UINT32_MAX));
	vision_pose.valid = sample.valid && (vision_pose.age_us <= VISION_POSE_TIMEOUT_US);
	return vision_pose;
}

void IIM42652::TelemetryStep(const hrt_abstime &cycle_begin, const MotorPwmWrite &motor_pwm_write,
			     const vision_pose_sample_t &vision_pose)
{
	TelemFrame frame{};
	const uint64_t now_us = static_cast<uint64_t>(cycle_begin);

	if ((_loop_state.last_cycle_start_us != 0U) && (now_us >= _loop_state.last_cycle_start_us)) {
		frame.loop_dt_us = static_cast<uint32_t>(now_us - _loop_state.last_cycle_start_us);
	}

	_loop_state.last_cycle_start_us = now_us;
	frame.exec_us = static_cast<uint32_t>(hrt_absolute_time() - cycle_begin);

	for (uint8_t i = 0; i < 3U; i++) {
		frame.accel_m_s2[i] = _latest_accel[i];
		frame.gyro_rad_s[i] = _latest_gyro[i];
	}

	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		frame.motor_pwm_us[i] = motor_pwm_write.motor_pwm_us[i];
	}

	for (uint8_t i = 0; i < 3U; ++i) {
		frame.vision_pos_m[i] = vision_pose.position_m[i];
		frame.vision_rpy_rad[i] = vision_pose.rpy_rad[i];
	}

	frame.vision_age_us = vision_pose.age_us;
	frame.vision_valid = vision_pose.valid ? 1U : 0U;

	_last_frame = frame;
	(void)telem_push(static_cast<uint64_t>(cycle_begin), &frame);
}

void IIM42652::PublishTelemetryOutsideIRQ()
{
	if (!_telem_pub.advertised()) {
		return;
	}

	for (uint8_t burst = 0; burst < TELEM_PUBLISH_BURST; ++burst) {
		telem_item_t item{};

		if (!telem_pop(&item)) {
			return;
		}

		const TelemFrame &frame = item.frame;
		rt_control_telemetry_s msg{};
		msg.timestamp = item.timestamp;
		msg.loop_dt_us = frame.loop_dt_us;
		msg.exec_us = frame.exec_us;
		msg.vision_age_us = frame.vision_age_us;
		msg.vision_valid = frame.vision_valid;

		for (int i = 0; i < 3; ++i) {
			msg.accel_m_s2[i] = frame.accel_m_s2[i];
			msg.gyro_rad_s[i] = frame.gyro_rad_s[i];
			msg.vision_pos_m[i] = frame.vision_pos_m[i];
			msg.vision_rpy_rad[i] = frame.vision_rpy_rad[i];
		}

		for (int i = 0; i < MOTOR_COUNT; ++i) {
			msg.motor_pwm_us[i] = frame.motor_pwm_us[i];
		}
		
		if (_telem_pub.publish(msg)) {
			_telem_pub_ok++;

		} else {
			_telem_pub_fail++;
		}
	}
}
