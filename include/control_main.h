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

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_NUM 4U

#define GRAVITY 9.80665f // [m/s^2]
#define ACCEL_SCALE (GRAVITY / 2048.0f) // [m/s^2/count]
#define GYRO_SCALE ((2000.0f / 32768.0f) * (3.14159265358979323846f / 180.0f)) // [rad/s/count]
#define TEMP_SENS 132.48f // [count/degC]
#define TEMP_OFFSET 25.0f // [degC]

typedef struct {
	uint8_t temp_hi;
	uint8_t temp_lo;
	uint8_t ax_hi;
	uint8_t ax_lo;
	uint8_t ay_hi;
	uint8_t ay_lo;
	uint8_t az_hi;
	uint8_t az_lo;
	uint8_t gx_hi;
	uint8_t gx_lo;
	uint8_t gy_hi;
	uint8_t gy_lo;
	uint8_t gz_hi;
	uint8_t gz_lo;
} imu_regs_t;

typedef struct {
	imu_regs_t imu;
	uint16_t pwm_min; // [us]
	uint16_t pwm_max; // [us]
} control_input_t;

typedef struct {
	float accel_raw[3]; // [count]
	float gyro_raw[3]; // [count]
	float accel[3]; // [m/s^2]
	float gyro[3]; // [rad/s]
	float temperature; // [degC]
	float motor[MOTOR_NUM]; // [0.0 ~ 1.0]
	uint16_t pwm[MOTOR_NUM]; // [us]
} control_output_t;

typedef struct {
	uint32_t freq; // [Hz]
	uint32_t period; // [us]
	float dt; // [s]
} control_timing_t;

extern const control_timing_t CONTROL_TIMING;

void control_step(const control_input_t *in, control_output_t *out);

#ifdef __cplusplus
}
#endif
