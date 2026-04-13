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

void IIM42652::UpdateLatestOptiSampleOutsideIRQ()
{
	vehicle_odometry_s odom{};

	if (!_vehicle_visual_odometry_sub.update(&odom)) {
		return;
	}

	LatestOpti sample{};
	sample.timestamp_sample = (odom.timestamp_sample != 0) ? odom.timestamp_sample : odom.timestamp;
	sample.seq = ++_opti_sample_counter;

	const bool position_valid = PX4_ISFINITE(odom.position[0])
				    && PX4_ISFINITE(odom.position[1])
				    && PX4_ISFINITE(odom.position[2]);
	const matrix::Quatf attitude_q(odom.q);
	const bool attitude_valid = attitude_q.isAllFinite();

	if (position_valid && attitude_valid) {
		const matrix::Eulerf attitude_euler(attitude_q);
		sample.x = odom.position[0];
		sample.y = odom.position[1];
		sample.z = odom.position[2];
		sample.roll = attitude_euler.phi();
		sample.pitch = attitude_euler.theta();
		sample.yaw = attitude_euler.psi();
		sample.valid = PX4_ISFINITE(sample.roll)
			       && PX4_ISFINITE(sample.pitch)
			       && PX4_ISFINITE(sample.yaw);

	} else {
		sample.x = NAN;
		sample.y = NAN;
		sample.z = NAN;
		sample.roll = NAN;
		sample.pitch = NAN;
		sample.yaw = NAN;
		sample.valid = false;
	}

	const uint32_t seq = _latest_opti_sample_seq.load();
	_latest_opti_sample_seq.store(seq + 1);
	_latest_opti = sample;
	_latest_opti_sample_seq.store(seq + 2);
}

bool IIM42652::CopyLatestOptiSample(LatestOpti &sample) const
{
	uint32_t seq_begin{0};
	uint32_t seq_end{0};

	do {
		seq_begin = _latest_opti_sample_seq.load();

		if (seq_begin == 0 || (seq_begin & 1u)) {
			return false;
		}

		sample = _latest_opti;
		seq_end = _latest_opti_sample_seq.load();

	} while ((seq_begin != seq_end) || (seq_end & 1u));

	return true;
}

opti_sample_t IIM42652::BuildOptiSample(const hrt_abstime &cycle_begin, const LatestOpti &sample) const
{
	opti_sample_t opti{};

	if (sample.seq == 0U) {
		return opti;
	}

	opti.x = sample.x;
	opti.y = sample.y;
	opti.z = sample.z;
	opti.roll = sample.roll;
	opti.pitch = sample.pitch;
	opti.yaw = sample.yaw;
	opti.seq = sample.seq;

	const uint64_t now_us = static_cast<uint64_t>(cycle_begin);
	const uint64_t age = (sample.timestamp_sample > 0 && now_us >= sample.timestamp_sample)
			     ? (now_us - sample.timestamp_sample)
			     : 0ULL;
	opti.age = static_cast<uint32_t>(math::min<uint64_t>(age, UINT32_MAX));
	opti.valid = sample.valid && (opti.age <= OPTI_TIMEOUT_US);
	return opti;
}

void IIM42652::TelemetryStep(const hrt_abstime &cycle_begin, uint32_t input, uint32_t control, uint32_t output,
			     const float motor[MOTOR_COUNT], const ActuatorWrite &actuator,
			     const opti_sample_t &opti)
{
	TelemFrame frame{};
	const uint64_t now_us = static_cast<uint64_t>(cycle_begin);
	const uint32_t period = CONTROL_TIMING.period;
	_loop_state.cycle++;
	frame.cycle = _loop_state.cycle;
	frame.ideal_start = ((double)(frame.cycle - 1U) * (double)period) * 1e-6;
	frame.actual_start = ((double)(now_us - _loop_state.t0)) * 1e-6;

	uint32_t loop_dt = 0U;

	if ((_loop_state.last_control_ts != 0U) && (now_us >= _loop_state.last_control_ts)) {
		loop_dt = static_cast<uint32_t>(now_us - _loop_state.last_control_ts);
	}

	_loop_state.last_control_ts = now_us;

	if (loop_dt > period) {
		const uint32_t skipped = (loop_dt / period) - 1U;
		_loop_state.deadline_miss_count += skipped;
	}

	if (loop_dt > _loop_state.max_loop_dt) {
		_loop_state.max_loop_dt = loop_dt;
	}

	frame.missed_cycles = _loop_state.deadline_miss_count;

	for (uint8_t i = 0; i < 3U; i++) {
		frame.accel[i] = _latest_accel[i];
		frame.gyro[i] = _latest_gyro[i];
	}

	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		frame.motor[i] = motor[i];
		frame.pwm[i] = actuator.pwm[i];
	}

	frame.input = input;
	frame.control = control;
	frame.output = output;
	frame.exec = static_cast<uint32_t>(hrt_absolute_time() - cycle_begin);
	frame.slack = (frame.exec < period) ? (period - frame.exec) : 0U;

	frame.opti_x = (opti.seq > 0U) ? opti.x : NAN;
	frame.opti_y = (opti.seq > 0U) ? opti.y : NAN;
	frame.opti_z = (opti.seq > 0U) ? opti.z : NAN;
	frame.opti_roll = (opti.seq > 0U) ? opti.roll : NAN;
	frame.opti_pitch = (opti.seq > 0U) ? opti.pitch : NAN;
	frame.opti_yaw = (opti.seq > 0U) ? opti.yaw : NAN;
	frame.opti_seq = opti.seq;
	frame.opti_age = opti.age;
	frame.opti_valid = opti.valid ? 1U : 0U;

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
		msg.cycle = frame.cycle;
		msg.actual_start_s = frame.actual_start;
		msg.exec_us = frame.exec;
		msg.input_us = frame.input;
		msg.control_us = frame.control;
		msg.output_us = frame.output;
		msg.missed_cycles = frame.missed_cycles;
		msg.opti_x = frame.opti_x;
		msg.opti_y = frame.opti_y;
		msg.opti_z = frame.opti_z;
		msg.opti_roll = frame.opti_roll;
		msg.opti_pitch = frame.opti_pitch;
		msg.opti_yaw = frame.opti_yaw;
		msg.opti_seq = frame.opti_seq;
		msg.opti_age_us = frame.opti_age;
		msg.opti_valid = frame.opti_valid;

		for (int i = 0; i < 3; ++i) {
			msg.accel_m_s2[i] = frame.accel[i];
			msg.gyro_rad_s[i] = frame.gyro[i];
		}

		for (int i = 0; i < MOTOR_COUNT; ++i) {
			msg.motor_norm[i] = frame.motor[i];
			msg.pwm_us[i] = frame.pwm[i];
		}
		
		if (_telem_pub.publish(msg)) {
			_telem_pub_ok++;

		} else {
			_telem_pub_fail++;
		}
	}
}
