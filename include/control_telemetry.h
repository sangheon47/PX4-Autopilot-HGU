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

#include "control_main.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEM_QUEUE_SIZE 512U
#define TELEM_PUBLISH_BURST 8U

typedef struct {
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	uint32_t seq;
	uint32_t age; // [us]
	bool valid;
} opti_sample_t;

typedef struct {
	uint32_t cycle;
	double ideal_start; // [s]
	double actual_start; // [s]
	uint32_t exec; // [us]
	uint32_t input; // [us]
	uint32_t control; // [us]
	uint32_t output; // [us]
	uint32_t slack; // [us]
	uint32_t missed_cycles;
	float accel[3]; // [m/s^2]
	float gyro[3]; // [rad/s]
	float motor[MOTOR_NUM]; // [0.0 ~ 1.0]
	uint16_t pwm[MOTOR_NUM]; // [us]
	float opti_x;
	float opti_y;
	float opti_z;
	float opti_roll;
	float opti_pitch;
	float opti_yaw;
	uint32_t opti_seq;
	uint32_t opti_age; // [us]
	uint8_t opti_valid;
} telem_frame_t;

typedef struct {
	uint32_t cycle;
	uint64_t t0; // [us]
	uint64_t last_control_ts; // [us]
	uint32_t deadline_miss_count;
	uint32_t max_loop_dt; // [us]
} loop_state_t;

typedef struct {
	uint64_t timestamp; // [us]
	telem_frame_t frame;
} telem_item_t;

typedef struct {
	uint32_t queued;
	uint32_t capacity;
	uint32_t dropped;
	uint32_t high_watermark;
} telem_queue_status_t;

void telem_reset(void);
bool telem_push(uint64_t timestamp, const telem_frame_t *frame);
bool telem_pop(telem_item_t *item);
void telem_status(telem_queue_status_t *status);

#ifdef __cplusplus
}
#endif
