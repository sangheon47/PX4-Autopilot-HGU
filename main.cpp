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
 * IMU sample in -> motor command -> 4 ESC PWM targets out.
 *
 * Change the realtime settings here, then build and upload.
 */

#include "include/control_main.h"

#include <assert.h>
#include <math.h>
#include <string.h>

#define SAMPLE_FREQ 200U    // [Hz]
#define SAMPLE_PERIOD 5000U // [us]
#define SAMPLE_DT 0.005f    // [s]

extern "C" const control_timing_t CONTROL_TIMING = {
	SAMPLE_FREQ,
	SAMPLE_PERIOD,
	SAMPLE_DT
};

static int16_t read_raw(uint8_t hi, uint8_t lo)
{
	// The IMU stores each sample as [high byte][low byte].
	return (int16_t)((hi << 8u) | lo);
}

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

	// 3. Direct PWM command for each motor.
	const uint16_t motor_1_pwm = in->pwm_min; // front-right [us]
	const uint16_t motor_2_pwm = in->pwm_min; // front-left  [us]
	const uint16_t motor_3_pwm = in->pwm_min; // rear-left   [us]
	const uint16_t motor_4_pwm = in->pwm_min; // rear-right  [us]

	const uint16_t pwm_cmd[MOTOR_NUM] = {
		motor_1_pwm,
		motor_2_pwm,
		motor_3_pwm,
		motor_4_pwm
	};

	// 4. Clamp PWM and keep a normalized copy for telemetry/status.
	const float pwm_span = (float)(in->pwm_max - in->pwm_min); // [us]

	for (uint8_t i = 0; i < MOTOR_NUM; ++i) {
		uint16_t pwm = pwm_cmd[i];

		if (pwm < in->pwm_min) {
			pwm = in->pwm_min;
		}

		if (pwm > in->pwm_max) {
			pwm = in->pwm_max;
		}

		out->pwm[i] = pwm; // [us]
		out->motor[i] = (pwm_span > 0.0f) ? ((float)(pwm - in->pwm_min) / pwm_span) : 0.0f; // [us] -> [0.0 ~ 1.0]
	}

}
