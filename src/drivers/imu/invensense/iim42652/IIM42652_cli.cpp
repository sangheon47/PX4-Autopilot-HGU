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

#include <drivers/drv_pwm_output.h>
#include <matrix/matrix/math.hpp>
#include <px4_arch/io_timer.h>

namespace
{
const char *motor_output_mode_str(uint8_t mode_raw)
{
	switch (mode_raw) {
	case 0:
		return "safe_idle";

	case 1:
		return "esc_cal_high";

	case 2:
		return "esc_cal_low";

	case 3:
		return "esc_test";

	case 4:
		return "auto";
	}

	return "unknown";
}
}

void IIM42652::print_status()
{
	const uint32_t control_period = CONTROL_TIMING.period;
	telem_queue_status_t queue{};
	telem_status(&queue);

	PX4_INFO("RT period_us=%u sample_time_s=%.6f freq_hz=%.1f telem_queue=%lu/%lu dropped=%lu high_water=%lu",
		 (unsigned)control_period,
		 (double)CONTROL_TIMING.dt,
		 (double)CONTROL_TIMING.freq,
		 (unsigned long)queue.queued,
		 (unsigned long)queue.capacity,
		 (unsigned long)queue.dropped,
		 (unsigned long)queue.high_watermark);

	PX4_INFO("RT pwm initialized=%s rate_hz=%u mask=0x%lx output_mode=%s test_pwm_us=%u",
		 _pwm_initialized ? "true" : "false",
		 MOTOR_PWM_RATE,
		 (unsigned long)_motor_pwm_mask,
		 motor_output_mode_str(_motor_output_mode.load()),
		 (unsigned)_esc_test_pwm.load());

	if (_last_frame.cycle == 0) {
		PX4_INFO("RT no cycle yet");
		return;
	}

	const double start_jitter_s = _last_frame.actual_start - _last_frame.ideal_start;
	const unsigned long overrun_us = (_last_frame.exec > control_period)
					 ? (unsigned long)(_last_frame.exec - control_period)
					 : 0UL;

	PX4_INFO("RT cycle=%lu ideal_start_s=%.6f actual_start_s=%.6f start_jitter_s=%+.6f",
		(unsigned long)_last_frame.cycle,
		_last_frame.ideal_start,
		_last_frame.actual_start,
		start_jitter_s);

	PX4_INFO("RT time input_us=%lu control_us=%lu output_us=%lu exec_us=%lu slack_us=%lu overrun_us=%lu",
		 (unsigned long)_last_frame.input,
		 (unsigned long)_last_frame.control,
		 (unsigned long)_last_frame.output,
		 (unsigned long)_last_frame.exec,
		 (unsigned long)_last_frame.slack,
		 overrun_us);

	PX4_INFO("RT accel_m_s2: x=%.5f y=%.5f z=%.5f gyro_rad_s: roll=%.5f pitch=%.5f yaw=%.5f",
		 (double)_last_frame.accel[0], (double)_last_frame.accel[1], (double)_last_frame.accel[2],
		 (double)_last_frame.gyro[0], (double)_last_frame.gyro[1], (double)_last_frame.gyro[2]);

	PX4_INFO("RT pwm_us=%u(%.2f%%) %u(%.2f%%) %u(%.2f%%) %u(%.2f%%)",
		 (unsigned)_last_frame.pwm[0], (double)(_last_frame.motor[0] * 100.f),
		 (unsigned)_last_frame.pwm[1], (double)(_last_frame.motor[1] * 100.f),
		 (unsigned)_last_frame.pwm[2], (double)(_last_frame.motor[2] * 100.f),
		 (unsigned)_last_frame.pwm[3], (double)(_last_frame.motor[3] * 100.f));

	if (_last_frame.opti_valid) {
		PX4_INFO("RT opti seq=%lu age_us=%lu pos=(%.4f, %.4f, %.4f) rpy=(%.4f, %.4f, %.4f)",
			 (unsigned long)_last_frame.opti_seq,
			 (unsigned long)_last_frame.opti_age,
			 (double)_last_frame.opti_x,
			 (double)_last_frame.opti_y,
			 (double)_last_frame.opti_z,
			 (double)_last_frame.opti_roll,
			 (double)_last_frame.opti_pitch,
			 (double)_last_frame.opti_yaw);

	} else {
		PX4_INFO("RT opti invalid seq=%lu age_us=%lu",
			 (unsigned long)_last_frame.opti_seq,
			 (unsigned long)_last_frame.opti_age);
	}

	PX4_INFO("RT perf bad_xfer=%lu fifo_empty=%lu fifo_overflow=%lu bad_reg=%lu failure_count=%u missed_cycles=%lu",
		 (unsigned long)perf_event_count(_bad_transfer_perf),
		 (unsigned long)perf_event_count(_fifo_empty_perf),
		 (unsigned long)perf_event_count(_fifo_overflow_perf),
		 (unsigned long)perf_event_count(_bad_register_perf),
		 (unsigned)_failure_count,
		 (unsigned long)_last_frame.missed_cycles);

	PX4_INFO("RT telem topic=rt_control_telemetry advertised=%s pub_ok=%lu pub_fail=%lu",
		 _telem_pub.advertised() ? "true" : "false",
		 (unsigned long)_telem_pub_ok,
		 (unsigned long)_telem_pub_fail);
}

void IIM42652::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case CLI_CUSTOM_RT_AUTO:
		_motor_output_mode.store(static_cast<uint8_t>(MotorOutputMode::Auto));
		PX4_INFO("RT main.cpp output enabled");
		break;

	case CLI_CUSTOM_ESC_CAL_HIGH:
		_motor_output_mode.store(static_cast<uint8_t>(MotorOutputMode::EscCalHigh));
		PX4_INFO("ESC calibration output: HIGH (%u us)", (unsigned)PWM_MAX_US);
		break;

	case CLI_CUSTOM_ESC_CAL_LOW:
		_motor_output_mode.store(static_cast<uint8_t>(MotorOutputMode::EscCalLow));

		if (_pwm_initialized) {
			for (int i = 0; i < MOTOR_COUNT; ++i) {
				up_pwm_servo_set(i, PWM_MIN_US);
			}

			up_pwm_update(_motor_pwm_mask);
		}

		PX4_INFO("ESC calibration output: LOW (%u us)", (unsigned)PWM_MIN_US);
		break;

	case CLI_CUSTOM_ESC_TEST: {
			const int requested_percent = math::constrain(cli.custom2, 0, (int)ESC_TEST_MAX_PERCENT);
			const uint16_t pwm = PWM_MIN_US + (uint16_t)(((uint32_t)(PWM_MAX_US - PWM_MIN_US) * (uint32_t)requested_percent) / 100U);
			_esc_test_pwm.store(pwm);
			_motor_output_mode.store(static_cast<uint8_t>(MotorOutputMode::EscTest));

			if (_pwm_initialized) {
				for (int i = 0; i < MOTOR_COUNT; ++i) {
					up_pwm_servo_set(i, pwm);
				}

				up_pwm_update(_motor_pwm_mask);
			}

			PX4_INFO("ESC test output: %d%% (%u us)", requested_percent, (unsigned)pwm);
		}
		break;

	case CLI_CUSTOM_ESC_CAL_STATUS:
	default:
		break;
	}

	PX4_INFO("ESC output status: output_mode=%s pwm_initialized=%s test_pwm_us=%u",
		 motor_output_mode_str(_motor_output_mode.load()),
		 _pwm_initialized ? "true" : "false",
		 (unsigned)_esc_test_pwm.load());
}
