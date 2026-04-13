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

namespace
{
struct DirectDataRegisterBuffer {
	uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::TEMP_DATA1) | DIR_READ};
	uint8_t temp_data1{0};
	uint8_t temp_data0{0};
	uint8_t accel_data_x1{0};
	uint8_t accel_data_x0{0};
	uint8_t accel_data_y1{0};
	uint8_t accel_data_y0{0};
	uint8_t accel_data_z1{0};
	uint8_t accel_data_z0{0};
	uint8_t gyro_data_x1{0};
	uint8_t gyro_data_x0{0};
	uint8_t gyro_data_y1{0};
	uint8_t gyro_data_y0{0};
	uint8_t gyro_data_z1{0};
	uint8_t gyro_data_z0{0};
	uint8_t tmst_fsync_h{0};
	uint8_t tmst_fsync_l{0};
	uint8_t int_status{0};
};
}

void IIM42652::ControlLoopTrampoline(void *arg)
{
	IIM42652 *self = static_cast<IIM42652 *>(arg);

	if (self != nullptr) {
		self->ControlLoopIRQ();
	}
}

void IIM42652::StartControlLoopIRQ()
{
	if (_control_loop_running) {
		return;
	}

	hrt_call_init(&_control_loop_call);
	_loop_state = {};
	telem_reset();
	_loop_state.t0 = static_cast<uint64_t>(hrt_absolute_time()) + CONTROL_TIMING.period;
	_loop_state.last_control_ts = 0U;
	_loop_state.deadline_miss_count = 0U;
	_request_reset = false;
	_fifo_flush_pending = false;
	_latest_publish_seq.store(0);
	_published_seq = 0;
	_telem_pub_ok = 0;
	_telem_pub_fail = 0;

	hrt_call_every(&_control_loop_call, CONTROL_TIMING.period, CONTROL_TIMING.period, ControlLoopTrampoline, this);
	_control_loop_running = true;
}

void IIM42652::StopControlLoopIRQ()
{
	if (_control_loop_running) {
		hrt_cancel(&_control_loop_call);
		_control_loop_running = false;
	}
}

void IIM42652::ControlLoopIRQ()
{
	if (_state != STATE::RT_LOOP_RUN) {
		return;
	}

	const hrt_abstime cycle_begin = hrt_absolute_time();
	const hrt_abstime timestamp_sample = cycle_begin;
	const bool sample_ok = ReadSampleDirect(timestamp_sample);
	const uint32_t input_time = static_cast<uint32_t>(hrt_absolute_time() - cycle_begin);

	if (sample_ok) {
		if (_failure_count > 0) {
			_failure_count--;
		}

	} else {
		_failure_count++;

		if (_failure_count > 10) {
			_request_reset = true;
		}
	}

	float motor[MOTOR_COUNT] {};
	LatestOpti opti_raw{};
	(void)CopyLatestOptiSample(opti_raw);
	const opti_sample_t opti = BuildOptiSample(cycle_begin, opti_raw);

	control_input_t in{};
	in.imu = _latest_imu;
	in.pwm_min = PWM_MIN_US;
	in.pwm_max = PWM_MAX_US;

	control_output_t out{};
	const hrt_abstime control_begin = hrt_absolute_time();
	control_step(&in, &out);
	const uint32_t control_time = static_cast<uint32_t>(hrt_absolute_time() - control_begin);

	for (int i = 0; i < 3; ++i) {
		_latest_accel[i] = out.accel[i];
		_latest_gyro[i] = out.gyro[i];
	}

	if (sample_ok) {
		const uint32_t seq = _latest_publish_seq.load();
		_latest_publish_seq.store(seq + 1);
		_latest_sample.timestamp_sample = timestamp_sample;
		_latest_sample.accel_raw[0] = out.accel_raw[0];
		_latest_sample.accel_raw[1] = out.accel_raw[1];
		_latest_sample.accel_raw[2] = out.accel_raw[2];
		_latest_sample.gyro_raw[0] = out.gyro_raw[0];
		_latest_sample.gyro_raw[1] = out.gyro_raw[1];
		_latest_sample.gyro_raw[2] = out.gyro_raw[2];
		_latest_sample.temperature = out.temperature;
		_latest_publish_seq.store(seq + 2);
	}

	for (int i = 0; i < MOTOR_COUNT; ++i) {
		motor[i] = out.motor[i];
	}

	ActuatorWrite actuator{};
	const hrt_abstime output_begin = hrt_absolute_time();
	WriteStep(out, actuator);
	const uint32_t output_time = static_cast<uint32_t>(hrt_absolute_time() - output_begin);

	TelemetryStep(cycle_begin, input_time, control_time, output_time, motor, actuator, opti);
}

bool IIM42652::ReadSampleDirect(const hrt_abstime &timestamp_sample)
{
	(void)timestamp_sample;
	DirectDataRegisterBuffer buffer{};
	SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_0);

	if (transfer(reinterpret_cast<uint8_t *>(&buffer), reinterpret_cast<uint8_t *>(&buffer), sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	_latest_imu.temp_hi = buffer.temp_data1;
	_latest_imu.temp_lo = buffer.temp_data0;
	_latest_imu.ax_hi = buffer.accel_data_x1;
	_latest_imu.ax_lo = buffer.accel_data_x0;
	_latest_imu.ay_hi = buffer.accel_data_y1;
	_latest_imu.ay_lo = buffer.accel_data_y0;
	_latest_imu.az_hi = buffer.accel_data_z1;
	_latest_imu.az_lo = buffer.accel_data_z0;
	_latest_imu.gx_hi = buffer.gyro_data_x1;
	_latest_imu.gx_lo = buffer.gyro_data_x0;
	_latest_imu.gy_hi = buffer.gyro_data_y1;
	_latest_imu.gy_lo = buffer.gyro_data_y0;
	_latest_imu.gz_hi = buffer.gyro_data_z1;
	_latest_imu.gz_lo = buffer.gyro_data_z0;

	return true;
}
