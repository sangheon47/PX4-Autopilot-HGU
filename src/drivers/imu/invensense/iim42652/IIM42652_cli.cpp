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
	telem_queue_status_t queue{};
	telem_status(&queue);

	PX4_INFO("RT sample_time_s=%.6f freq_hz=%.1f telem_queue=%lu/%lu dropped=%lu high_water=%lu",
		 (double)CONTROL_TIMING.dt,
		 (double)CONTROL_TIMING.freq,
		 (unsigned long)queue.queued,
		 (unsigned long)queue.capacity,
		 (unsigned long)queue.dropped,
		 (unsigned long)queue.high_watermark);

	PX4_INFO("RT motor_pwm initialized=%s rate_hz=%u mask=0x%lx output_mode=%s test_pwm_us=%u",
		 _motor_pwm_initialized ? "true" : "false",
		 MOTOR_PWM_RATE,
		 (unsigned long)_motor_pwm_mask,
		 motor_output_mode_str(_motor_output_mode.load()),
		 (unsigned)_esc_test_pwm.load());

	if ((_last_frame.loop_dt_us == 0U) && (_last_frame.exec_us == 0U)) {
		PX4_INFO("RT no cycle yet");
		return;
	}

	PX4_INFO("RT time loop_dt_us=%lu exec_us=%lu",
		 (unsigned long)_last_frame.loop_dt_us,
		 (unsigned long)_last_frame.exec_us);

	PX4_INFO("RT accel_m_s2: x=%.5f y=%.5f z=%.5f gyro_rad_s: roll=%.5f pitch=%.5f yaw=%.5f",
		 (double)_last_frame.accel_m_s2[0], (double)_last_frame.accel_m_s2[1], (double)_last_frame.accel_m_s2[2],
		 (double)_last_frame.gyro_rad_s[0], (double)_last_frame.gyro_rad_s[1], (double)_last_frame.gyro_rad_s[2]);

	PX4_INFO("RT motor_pwm_us=%u %u %u %u",
		 (unsigned)_last_frame.motor_pwm_us[0],
		 (unsigned)_last_frame.motor_pwm_us[1],
		 (unsigned)_last_frame.motor_pwm_us[2],
		 (unsigned)_last_frame.motor_pwm_us[3]);

	if (_last_frame.vision_valid) {
		PX4_INFO("RT vision age_us=%lu pos_m=(%.4f, %.4f, %.4f) rpy_rad=(%.4f, %.4f, %.4f)",
			 (unsigned long)_last_frame.vision_age_us,
			 (double)_last_frame.vision_pos_m[0],
			 (double)_last_frame.vision_pos_m[1],
			 (double)_last_frame.vision_pos_m[2],
			 (double)_last_frame.vision_rpy_rad[0],
			 (double)_last_frame.vision_rpy_rad[1],
			 (double)_last_frame.vision_rpy_rad[2]);

	} else {
		PX4_INFO("RT vision invalid age_us=%lu",
			 (unsigned long)_last_frame.vision_age_us);
	}

	PX4_INFO("RT perf bad_xfer=%lu fifo_empty=%lu fifo_overflow=%lu bad_reg=%lu failure_count=%u",
		 (unsigned long)perf_event_count(_bad_transfer_perf),
		 (unsigned long)perf_event_count(_fifo_empty_perf),
		 (unsigned long)perf_event_count(_fifo_overflow_perf),
		 (unsigned long)perf_event_count(_bad_register_perf),
		 (unsigned)_failure_count);

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

		if (_motor_pwm_initialized) {
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

			if (_motor_pwm_initialized) {
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
		 _motor_pwm_initialized ? "true" : "false",
		 (unsigned)_esc_test_pwm.load());
}
