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
	float position_m[3]; // [m]
	float rpy_rad[3]; // [rad]
	uint32_t age_us; // [us]
	bool valid;
} vision_pose_sample_t;

typedef struct {
	uint32_t loop_dt_us; // [us]
	uint32_t exec_us; // [us]
	float accel_m_s2[3]; // [m/s^2]
	float gyro_rad_s[3]; // [rad/s]
	uint16_t motor_pwm_us[MOTOR_NUM]; // [us]
	float vision_pos_m[3]; // [m]
	float vision_rpy_rad[3]; // [rad]
	uint32_t vision_age_us; // [us]
	uint8_t vision_valid;
} telem_frame_t;

typedef struct {
	uint64_t last_cycle_start_us; // [us]
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
