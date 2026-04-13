/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/**
 * @file main.cpp
 *
 * Open this file first.
 * This is the visible 7-nano realtime loop:
 * IMU sample in -> motor PWM command -> 4 ESC PWM targets out.
 *
 * Change the realtime settings here, then build and upload.
 */

#include "include/control_main.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define SAMPLE_FREQ 200U // [Hz]
// SAMPLE_PERIOD is an integer [us] scheduler period, so choose SAMPLE_FREQ accordingly for exact timing.
#define SAMPLE_PERIOD (1000000U / SAMPLE_FREQ) // [us]
#define SAMPLE_DT ((float)(SAMPLE_PERIOD) * 1e-6f) // [s]

extern "C" const control_timing_t CONTROL_TIMING = {
	SAMPLE_FREQ,
	SAMPLE_PERIOD,
	SAMPLE_DT
};

static int16_t read_raw(uint8_t hi, uint8_t lo);

extern "C" void control_step(const control_input_t *in, control_output_t *out)
{
	assert(in != NULL);
	assert(out != NULL);

	memset(out, 0, sizeof(*out));
	const imu_regs_t &imu = in->imu;

	// 1. Decode the latest raw IMU registers into signed sensor counts.
	const int16_t ax_raw = read_raw(imu.ax_hi, imu.ax_lo);
	const int16_t ay_raw = read_raw(imu.ay_hi, imu.ay_lo);
	const int16_t az_raw = read_raw(imu.az_hi, imu.az_lo);
	const int16_t gx_raw = read_raw(imu.gx_hi, imu.gx_lo);
	const int16_t gy_raw = read_raw(imu.gy_hi, imu.gy_lo);
	const int16_t gz_raw = read_raw(imu.gz_hi, imu.gz_lo);
	const int16_t temp_raw = read_raw(imu.temp_hi, imu.temp_lo);

	out->accel_raw[0] = (float)ax_raw;
	out->accel_raw[1] = (float)((ay_raw == INT16_MIN) ? INT16_MAX : -ay_raw);
	out->accel_raw[2] = (float)((az_raw == INT16_MIN) ? INT16_MAX : -az_raw);
	out->gyro_raw[0] = (float)gx_raw;
	out->gyro_raw[1] = (float)((gy_raw == INT16_MIN) ? INT16_MAX : -gy_raw);
	out->gyro_raw[2] = (float)((gz_raw == INT16_MIN) ? INT16_MAX : -gz_raw);
	out->temperature = ((float)temp_raw / TEMP_SENS) + TEMP_OFFSET; // [count] -> [degC]

	// 2. Convert raw counts into the physical units used by the control law.
	for (uint8_t i = 0; i < 3; ++i) {
		out->accel[i] = out->accel_raw[i] * ACCEL_SCALE; // [count] -> [m/s^2]
		out->gyro[i] = out->gyro_raw[i] * GYRO_SCALE; // [count] -> [rad/s]
	}

	// 3. Direct motor PWM command for each ESC output.
	const float motor1_pwm_pct = 0.0f; // [%] front-right
	const float motor2_pwm_pct = 0.0f; // [%] front-left
	const float motor3_pwm_pct = 0.0f; // [%] rear-left
	const float motor4_pwm_pct = 0.0f; // [%] rear-right

	const float motor_pwm_cmd_pct[MOTOR_NUM] = {
		motor1_pwm_pct,
		motor2_pwm_pct,
		motor3_pwm_pct,
		motor4_pwm_pct
	};

	// 4. Clamp motor PWM percent and convert it into the ESC pulse width.
	const float motor_pwm_span_us = (float)(in->motor_pwm_max_us - in->motor_pwm_min_us); // [us]

	for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
		float motor_pwm_pct = motor_pwm_cmd_pct[i];

		if (motor_pwm_pct < 0.0f) {
			motor_pwm_pct = 0.0f;
		}

		if (motor_pwm_pct > 100.0f) {
			motor_pwm_pct = 100.0f;
		}

		const uint16_t motor_pwm_us = (uint16_t)roundf((float)in->motor_pwm_min_us
					 + (motor_pwm_span_us * motor_pwm_pct * 0.01f)); // [%] -> [us]

		out->motor_pwm_pct[i] = motor_pwm_pct; // [%]
		out->motor_pwm_us[i] = motor_pwm_us; // [us]
	}

}

static int16_t read_raw(uint8_t hi, uint8_t lo)
{
	// The IMU stores each sample as [high byte][low byte].
	return (int16_t)((hi << 8u) | lo);
}
