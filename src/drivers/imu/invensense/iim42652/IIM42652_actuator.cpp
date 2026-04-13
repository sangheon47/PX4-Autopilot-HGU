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

void IIM42652::WriteStep(const control_output_t &out_cmd, ActuatorWrite &out)
{
	const MotorOutputMode output_mode = static_cast<MotorOutputMode>(_motor_output_mode.load());
	uint16_t pwm = PWM_MIN_US;

	if (output_mode == MotorOutputMode::EscCalHigh) {
		pwm = PWM_MAX_US;

	} else if (output_mode == MotorOutputMode::EscTest) {
		pwm = _esc_test_pwm.load();
	}

	for (int i = 0; i < MOTOR_COUNT; i++) {
		if (output_mode == MotorOutputMode::Auto) {
			pwm = math::constrain(out_cmd.pwm[i], PWM_MIN_US, PWM_MAX_US);
		}

		out.pwm[i] = pwm;
		up_pwm_servo_set(i, pwm);
	}

	up_pwm_update(_motor_pwm_mask);
}

bool IIM42652::InitActuatorDirect()
{
	if (_pwm_initialized) {
		return true;
	}

	if (up_pwm_servo_init(_motor_pwm_mask) < 0) {
		return false;
	}

	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		if ((up_pwm_servo_get_rate_group(timer) & _motor_pwm_mask) == 0) {
			continue;
		}

		if (up_pwm_servo_set_rate_group_update(timer, MOTOR_PWM_RATE) < 0) {
			up_pwm_servo_deinit(_motor_pwm_mask);
			PX4_ERR("pwm rate init failed (timer=%d rate=%u)", timer, MOTOR_PWM_RATE);
			return false;
		}
	}

	for (int i = 0; i < MOTOR_COUNT; ++i) {
		up_pwm_servo_set(i, PWM_MIN_US);
	}

	up_pwm_servo_arm(true, _motor_pwm_mask);

	for (int i = 0; i < MOTOR_COUNT; ++i) {
		up_pwm_servo_set(i, PWM_MIN_US);
	}

	up_pwm_update(_motor_pwm_mask);
	_pwm_initialized = true;
	return true;
}

void IIM42652::DeinitActuatorDirect()
{
	if (_pwm_initialized) {
		up_pwm_servo_arm(false, _motor_pwm_mask);
		up_pwm_servo_deinit(_motor_pwm_mask);
		_pwm_initialized = false;
	}
}
