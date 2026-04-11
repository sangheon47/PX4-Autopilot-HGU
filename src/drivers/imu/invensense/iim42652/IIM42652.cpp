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

#include <math.h>
#include <string.h>

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static constexpr uint16_t combine_uint(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

static const char *motor_output_mode_str(uint8_t mode_raw)
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
	}

	return "unknown";
}

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

IIM42652::IIM42652(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (config.drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	if (config.custom1 != 0) {
		_enable_clock_input = true;
		_input_clock_freq = config.custom1;
		ConfigureCLKIN();

	} else {
		_enable_clock_input = false;
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

IIM42652::~IIM42652()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int IIM42652::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		// DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	if (!InitActuatorDirect()) {
		PX4_ERR("actuator direct init failed");
		return PX4_ERROR;
	}

	if (!_rt_control_telem_pub.advertise()) {
		PX4_WARN("rt_control_telemetry advertise failed");
	}

	return Reset() ? 0 : -1;
}

bool IIM42652::Reset()
{
	StopControlLoopIRQ();
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void IIM42652::exit_and_cleanup()
{
	StopControlLoopIRQ();
	_rt_control_telem_pub.unadvertise();
	DeinitActuatorDirect();
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void IIM42652::print_status()
{
	PX4_INFO("RT period_us=%u, freq_hz=%.1f",
		 (unsigned)CONTROL_PERIOD_US,
		 (double)(1000000.0 / (double)CONTROL_PERIOD_US));

	PX4_INFO("RT pwm initialized=%s rate_hz=%u mask=0x%lx output_mode=%s test_pwm_us=%u",
		 _pwm_initialized ? "true" : "false",
		 MOTOR_PWM_RATE,
		 (unsigned long)_motor_pwm_mask,
		 motor_output_mode_str(_motor_output_mode.load()),
		 (unsigned)_esc_test_pwm_us.load());

	if (_last_frame.cycle == 0) {
		PX4_INFO("RT no cycle yet");
		return;
	}

	const double start_jitter_s = _last_frame.actual_start_s - _last_frame.ideal_start_s;
	const unsigned long overrun_us = (_last_frame.exec_us > CONTROL_PERIOD_US)
					 ? (unsigned long)(_last_frame.exec_us - CONTROL_PERIOD_US)
					 : 0UL;

	PX4_INFO("RT cycle=%lu ideal_start_s=%.6f actual_start_s=%.6f start_jitter_s=%+.6f",
		(unsigned long)_last_frame.cycle,
		_last_frame.ideal_start_s,
		_last_frame.actual_start_s,
		start_jitter_s);

	PX4_INFO("RT time input_us=%lu control_us=%lu output_us=%lu exec_us=%lu slack_us=%lu overrun_us=%lu",
		 (unsigned long)_last_frame.input_us,
		 (unsigned long)_last_frame.control_us,
		 (unsigned long)_last_frame.output_us,
		 (unsigned long)_last_frame.exec_us,
		 (unsigned long)_last_frame.slack_us,
		 overrun_us);

	PX4_INFO("RT accel_m_s2: x=%.5f y=%.5f z=%.5f gyro_rad_s: roll=%.5f pitch=%.5f yaw=%.5f",
		 (double)_last_frame.accel_m_s2[0], (double)_last_frame.accel_m_s2[1], (double)_last_frame.accel_m_s2[2],
		 (double)_last_frame.gyro_rad_s[0], (double)_last_frame.gyro_rad_s[1], (double)_last_frame.gyro_rad_s[2]);

	rt_controller_debug_state_t debug_state{};
	rt_controller_get_debug_state(&debug_state);
	PX4_INFO("RT tilt_deg init=%u raw=(y=%.2f z=%.2f) rel=(y=%.2f z=%.2f) ref=(y=%.2f z=%.2f) ref_valid=(y=%u z=%u)",
		 (unsigned)debug_state.initialized,
		 (double)debug_state.tilt_y_deg,
		 (double)debug_state.tilt_z_deg,
		 (double)debug_state.tilt_y_rel_deg,
		 (double)debug_state.tilt_z_rel_deg,
		 (double)debug_state.tilt_y_ref_deg,
		 (double)debug_state.tilt_z_ref_deg,
		 (unsigned)debug_state.tilt_y_ref_initialized,
		 (unsigned)debug_state.tilt_z_ref_initialized);

	PX4_INFO("RT pwm_us=%u(%.2f%%) %u(%.2f%%) %u(%.2f%%) %u(%.2f%%)",
		 (unsigned)_last_frame.pwm_us[0], (double)(_last_frame.motor_norm[0] * 100.f),
		 (unsigned)_last_frame.pwm_us[1], (double)(_last_frame.motor_norm[1] * 100.f),
		 (unsigned)_last_frame.pwm_us[2], (double)(_last_frame.motor_norm[2] * 100.f),
		 (unsigned)_last_frame.pwm_us[3], (double)(_last_frame.motor_norm[3] * 100.f));

	if (_last_frame.opti_valid) {
		PX4_INFO("RT opti seq=%lu age_us=%lu pos=(%.4f, %.4f, %.4f) rpy=(%.4f, %.4f, %.4f)",
			 (unsigned long)_last_frame.opti_seq,
			 (unsigned long)_last_frame.opti_age_us,
			 (double)_last_frame.opti_x,
			 (double)_last_frame.opti_y,
			 (double)_last_frame.opti_z,
			 (double)_last_frame.opti_roll,
			 (double)_last_frame.opti_pitch,
			 (double)_last_frame.opti_yaw);

	} else {
		PX4_INFO("RT opti invalid seq=%lu age_us=%lu",
			 (unsigned long)_last_frame.opti_seq,
			 (unsigned long)_last_frame.opti_age_us);
	}

	PX4_INFO("RT perf bad_xfer=%lu fifo_empty=%lu fifo_overflow=%lu bad_reg=%lu failure_count=%u missed_cycles=%lu",
		 (unsigned long)perf_event_count(_bad_transfer_perf),
		 (unsigned long)perf_event_count(_fifo_empty_perf),
		 (unsigned long)perf_event_count(_fifo_overflow_perf),
		 (unsigned long)perf_event_count(_bad_register_perf),
		 (unsigned)_failure_count,
		 (unsigned long)_last_frame.missed_cycles);

	PX4_INFO("RT telem topic=rt_control_telemetry advertised=%s pub_ok=%lu pub_fail=%lu",
		 _rt_control_telem_pub.advertised() ? "true" : "false",
		 (unsigned long)_telem_publish_count,
		 (unsigned long)_telem_publish_fail_count);
}

void IIM42652::custom_method(const BusCLIArguments &cli)
{
	switch (cli.custom1) {
	case CLI_CUSTOM_RT_ZERO:
		rt_controller_zero_reference();
		PX4_INFO("RT controller reference zeroed");
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
			_esc_test_pwm_us.store(pwm);
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
		 (unsigned)_esc_test_pwm_us.load());
}

int IIM42652::probe()
{
	for (int i = 0; i < 3; i++) {
		uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (whoami == WHOAMI) {
			return PX4_OK;

		} else {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);

			uint8_t reg_bank_sel = RegisterRead(Register::BANK_0::REG_BANK_SEL);
			int bank = reg_bank_sel >> 4;

			if (bank >= 1 && bank <= 3) {
				DEVICE_DEBUG("incorrect register bank for WHO_AM_I REG_BANK_SEL:0x%02x, bank:%d", reg_bank_sel, bank);
				// force bank selection and retry
				SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_0, true);
			}
		}
	}

	return PX4_ERROR;
}

void IIM42652::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// DEVICE_CONFIG: Software reset configuration
		RegisterWrite(Register::BANK_0::DEVICE_CONFIG, DEVICE_CONFIG_BIT::SOFT_RESET_CONFIG);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(1_ms); // wait 1 ms for soft reset to be effective
		break;

	case STATE::WAIT_FOR_RESET:
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::BANK_0::DEVICE_CONFIG) == 0x00)
		    && (RegisterRead(Register::BANK_0::INT_STATUS) & INT_STATUS_BIT::RESET_DONE_INT)) {

			// Wakeup accel and gyro and schedule remaining configuration
			RegisterWrite(Register::BANK_0::PWR_MGMT0, PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE);
			_state = STATE::CONFIGURE;
			ScheduleDelayed(30_ms); // 30 ms gyro startup time, 10 ms accel from sleep to valid data

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then reset the FIFO
			_state = STATE::RT_LOOP_INIT;
			ScheduleDelayed(1_ms);

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::RT_LOOP_INIT:

			_state = STATE::RT_LOOP_RUN;
			FIFOReset();
			DataReadyInterruptDisable();
			_data_ready_interrupt_enabled = false;
			StartControlLoopIRQ();
			ScheduleDelayed(100_ms); // housekeeping + telemetry flush

			break;

	case STATE::RT_LOOP_RUN: {
					if (_control_loop_running) {
						UpdateLatestOptiSampleOutsideIRQ();
						PublishSampleOutsideIRQ();
						PublishTelemetryOutsideIRQ();

					if (_request_reset) {
						_request_reset = false;
						Reset();
						return;
					}

					if (_fifo_flush_pending) {
						_fifo_flush_pending = false;
						StopControlLoopIRQ();
						FIFOReset();
						StartControlLoopIRQ();
					}

						ScheduleDelayed(CONTROL_PERIOD_US);
						break;
					}

				hrt_abstime timestamp_sample = now;
				uint8_t samples = 0;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;
					samples = _fifo_gyro_samples;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			if (samples == 0) {
				// check current FIFO count
				const uint16_t fifo_count = FIFOReadCount();

				if (fifo_count >= FIFO::SIZE) {
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (fifo_count == 0) {
					perf_count(_fifo_empty_perf);

				} else {
					// FIFO count (size in bytes)
					samples = (fifo_count / sizeof(FIFO::DATA));

					// tolerate minor jitter, leave sample to next iteration if behind by only 1
					if (samples == _fifo_gyro_samples + 1) {
						timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
						samples--;
					}

					if (samples > FIFO_MAX_SAMPLES) {
						// not technically an overflow, but more samples than we expected or can publish
						FIFOReset();
						perf_count(_fifo_overflow_perf);
						samples = 0;
					}
				}
			}

			bool success = false;

			if (samples >= 1) {
				if (FIFORead(timestamp_sample, samples)) {
					success = true;

					if (_failure_count > 0) {
						_failure_count--;
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
				    && RegisterCheck(_register_bank1_cfg[_checked_register_bank1])
				    && RegisterCheck(_register_bank2_cfg[_checked_register_bank2])
				   ) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
					_checked_register_bank1 = (_checked_register_bank1 + 1) % size_register_bank1_cfg;
					_checked_register_bank2 = (_checked_register_bank2 + 1) % size_register_bank2_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}
			}
		}

		break;
	}
}

void IIM42652::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt
	const float min_interval = FIFO_SAMPLE_DT;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);

	ConfigureFIFOWatermark(_fifo_gyro_samples);
}

void IIM42652::ConfigureFIFOWatermark(uint8_t samples)
{
	// FIFO watermark threshold in number of bytes
	const uint16_t fifo_watermark_threshold = samples * sizeof(FIFO::DATA);

	for (auto &r : _register_bank0_cfg) {
		if (r.reg == Register::BANK_0::FIFO_CONFIG2) {
			// FIFO_WM[7:0]  FIFO_CONFIG2
			r.set_bits = fifo_watermark_threshold & 0xFF;

		} else if (r.reg == Register::BANK_0::FIFO_CONFIG3) {
			// FIFO_WM[11:8] FIFO_CONFIG3
			r.set_bits = (fifo_watermark_threshold >> 8) & 0x0F;
		}
	}
}

void IIM42652::ConfigureCLKIN()
{
	for (auto &r0 : _register_bank0_cfg) {
		if (r0.reg == Register::BANK_0::INTF_CONFIG1) {
			r0.set_bits = r0.set_bits | INTF_CONFIG1_BIT::RTC_MODE;
		}
	}

	for (auto &r1 : _register_bank1_cfg) {
		if (r1.reg == Register::BANK_1::INTF_CONFIG5) {
			r1.set_bits = INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_SET;
			r1.clear_bits = INTF_CONFIG5_BIT::PIN9_FUNCTION_CLKIN_CLEAR;
		}
	}
}

void IIM42652::SelectRegisterBank(enum REG_BANK_SEL_BIT bank, bool force)
{
	if (bank != _last_register_bank || force) {
		// select BANK_0
		uint8_t cmd_bank_sel[2] {};
		cmd_bank_sel[0] = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = bank;
		transfer(cmd_bank_sel, cmd_bank_sel, sizeof(cmd_bank_sel));

		_last_register_bank = bank;
	}
}

bool IIM42652::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank1_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank1_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	// 20-bits data format used
	//  the only FSR settings that are operational are ±2000dps for gyroscope and ±16g for accelerometer
	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
	_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
	_px4_gyro.set_range(math::radians(2000.f));
	_px4_gyro.set_scale(math::radians(2000.f / 32768.f));

	return success;
}

int IIM42652::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<IIM42652 *>(arg)->DataReady();
	return 0;
}

void IIM42652::DataReady()
{
	_drdy_timestamp_sample.store(hrt_absolute_time());
	ScheduleNow();
}

bool IIM42652::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool IIM42652::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

template <typename T>
bool IIM42652::RegisterCheck(const T &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

template <typename T>
uint8_t IIM42652::RegisterRead(T reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	SelectRegisterBank(reg);
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

template <typename T>
void IIM42652::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	SelectRegisterBank(reg);
	transfer(cmd, cmd, sizeof(cmd));
}

template <typename T>
void IIM42652::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t IIM42652::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ;
	SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_0);

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool IIM42652::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 4, FIFO::SIZE);
	SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_0);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	if (buffer.INT_STATUS & INT_STATUS_BIT::FIFO_FULL_INT) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint16_t fifo_count_bytes = combine(buffer.FIFO_COUNTH, buffer.FIFO_COUNTL);

	if (fifo_count_bytes >= FIFO::SIZE) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint8_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	// check FIFO header in every sample
	uint8_t valid_samples = 0;

	for (int i = 0; i < math::min(samples, fifo_count_samples); i++) {
		bool valid = true;

		// With FIFO_ACCEL_EN and FIFO_GYRO_EN header should be 8’b_0110_10xx
		const uint8_t FIFO_HEADER = buffer.f[i].FIFO_Header;

		if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_MSG) {
			// FIFO sample empty if HEADER_MSG set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ACCEL)) {
			// accel bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_GYRO)) {
			// gyro bit not set
			valid = false;

		} else if (!(FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_20)) {
			// Packet does not contain a new and valid extended 20-bit data
			valid = false;

		} else if ((FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_TIMESTAMP_FSYNC) != Bit3) {
			// Packet does not contain ODR timestamp
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_ACCEL) {
			// accel ODR changed
			valid = false;

		} else if (FIFO_HEADER & FIFO::FIFO_HEADER_BIT::HEADER_ODR_GYRO) {
			// gyro ODR changed
			valid = false;
		}

		if (valid) {
			valid_samples++;

		} else {
			perf_count(_bad_transfer_perf);
			break;
		}
	}

	if (valid_samples > 0) {
		if (ProcessTemperature(buffer.f, valid_samples)) {
			ProcessGyro(timestamp_sample, buffer.f, valid_samples);
			ProcessAccel(timestamp_sample, buffer.f, valid_samples);

			return true;
		}
	}

	return false;
}

void IIM42652::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// SIGNAL_PATH_RESET: FIFO flush
	RegisterSetBits(Register::BANK_0::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::FIFO_FLUSH);

	// reset while FIFO is disabled
	_drdy_timestamp_sample.store(0);
}

static constexpr int32_t reassemble_20bit(const uint32_t a, const uint32_t b, const uint32_t c)
{
	// 0xXXXAABBC
	uint32_t high   = ((a << 12) & 0x000FF000);
	uint32_t low    = ((b << 4)  & 0x00000FF0);
	uint32_t lowest = (c         & 0x0000000F);

	uint32_t x = high | low | lowest;

	if (a & Bit7) {
		// sign extend
		x |= 0xFFF00000u;
	}

	return static_cast<int32_t>(x);
}

void IIM42652::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;

	// 18-bits of accelerometer data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {

		uint16_t timestamp_fifo = combine_uint(fifo[i].TimeStamp_h, fifo[i].TimeStamp_l);

		if (_enable_clock_input) {
			accel.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);

		} else {
			accel.dt = (float)timestamp_fifo * FIFO_TIMESTAMP_SCALING;
		}

		// 20 bit hires mode
		// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
		// Accel data is 18 bit ()
		int32_t accel_x = reassemble_20bit(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0,
						   (fifo[i].Ext_Accel_X_Gyro_X & 0xF0) >> 4);
		int32_t accel_y = reassemble_20bit(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0,
						   (fifo[i].Ext_Accel_Y_Gyro_Y & 0xF0) >> 4);
		int32_t accel_z = reassemble_20bit(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0,
						   (fifo[i].Ext_Accel_Z_Gyro_Z & 0xF0) >> 4);

		// sample invalid if -524288
		if (accel_x != -524288 && accel_y != -524288 && accel_z != -524288) {
			// check if any values are going to exceed int16 limits
			static constexpr int16_t max_accel = INT16_MAX;
			static constexpr int16_t min_accel = INT16_MIN;

			if (accel_x >= max_accel || accel_x <= min_accel) {
				scale_20bit = true;
			}

			if (accel_y >= max_accel || accel_y <= min_accel) {
				scale_20bit = true;
			}

			if (accel_z >= max_accel || accel_z <= min_accel) {
				scale_20bit = true;
			}

			// shift by 2 (2 least significant bits are always 0)
			accel.x[accel.samples] = accel_x / 4;
			accel.y[accel.samples] = accel_y / 4;
			accel.z[accel.samples] = accel_z / 4;
			accel.samples++;
		}
	}

	if (!scale_20bit) {
		// if highres enabled accel data is always 8192 LSB/g
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			// 20 bit hires mode
			// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
			// Accel data is 18 bit ()
			int16_t accel_x = combine(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0);
			int16_t accel_y = combine(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0);
			int16_t accel_z = combine(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0);

			accel.x[i] = accel_x;
			accel.y[i] = accel_y;
			accel.z[i] = accel_z;
		}

		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
	}

	const float accel_scale = scale_20bit ? (CONSTANTS_ONE_G / 2048.f) : (CONSTANTS_ONE_G / 8192.f);

	// correct frame for publication
	for (int i = 0; i < accel.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel.x[i];
		accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
		accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		const int last = accel.samples - 1;
		// 여기서 사용한 실제 scale 값(accel_scale)을 곱해서 SI 단위로 저장
		_latest_accel_m_s2[0] = accel.x[last] * accel_scale;
		_latest_accel_m_s2[1] = accel.y[last] * accel_scale;
		_latest_accel_m_s2[2] = accel.z[last] * accel_scale;
	}
}

void IIM42652::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;

	// 20-bits of gyroscope data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {

		uint16_t timestamp_fifo = combine_uint(fifo[i].TimeStamp_h, fifo[i].TimeStamp_l);

		if (_enable_clock_input) {
			gyro.dt = (float)timestamp_fifo * ((1.f / _input_clock_freq) * 1e6f);

		} else {
			gyro.dt = (float)timestamp_fifo * FIFO_TIMESTAMP_SCALING;
		}

		// 20 bit hires mode
		// Gyro [19:12] + Gyro [11:4] + Gyro [3:0] (bottom 4 bits of 20 bit extension byte)
		int32_t gyro_x = reassemble_20bit(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0, fifo[i].Ext_Accel_X_Gyro_X & 0x0F);
		int32_t gyro_y = reassemble_20bit(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0, fifo[i].Ext_Accel_Y_Gyro_Y & 0x0F);
		int32_t gyro_z = reassemble_20bit(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0, fifo[i].Ext_Accel_Z_Gyro_Z & 0x0F);

		// check if any values are going to exceed int16 limits
		static constexpr int16_t max_gyro = INT16_MAX;
		static constexpr int16_t min_gyro = INT16_MIN;

		if (gyro_x >= max_gyro || gyro_x <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_y >= max_gyro || gyro_y <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_z >= max_gyro || gyro_z <= min_gyro) {
			scale_20bit = true;
		}

		gyro.x[gyro.samples] = gyro_x / 2;
		gyro.y[gyro.samples] = gyro_y / 2;
		gyro.z[gyro.samples] = gyro_z / 2;
		gyro.samples++;
	}

	if (!scale_20bit) {
		// if highres enabled gyro data is always 131 LSB/dps
		_px4_gyro.set_scale(math::radians(1.f / 131.f));

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			gyro.x[i] = combine(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0);
			gyro.y[i] = combine(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0);
			gyro.z[i] = combine(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0);
		}

		_px4_gyro.set_scale(math::radians(2000.f / 32768.f));
	}

	const float gyro_scale = scale_20bit ? math::radians(2000.f / 32768.f) : math::radians(1.f / 131.f);

	// correct frame for publication
	for (int i = 0; i < gyro.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro.x[i];
		gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
		gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (gyro.samples > 0) {
		const int last = gyro.samples - 1;
		// 여기서 사용한 실제 scale 값(gyro_scale)을 곱해서 SI 단위로 저장
		_latest_gyro_rad_s[0] = gyro.x[last] * gyro_scale;
		_latest_gyro_rad_s[1] = gyro.y[last] * gyro_scale;
		_latest_gyro_rad_s[2] = gyro.z[last] * gyro_scale;
	}
}

bool IIM42652::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature[FIFO_MAX_SAMPLES];
	float temperature_sum{0};

	int valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		const int16_t t = combine(fifo[i].TEMP_DATA1, fifo[i].TEMP_DATA0);

		// sample invalid if -32768
		if (t != -32768) {
			temperature_sum += t;
			temperature[valid_samples] = t;
			valid_samples++;
		}
	}

	if (valid_samples > 0) {
		const float temperature_avg = temperature_sum / valid_samples;

		for (int i = 0; i < valid_samples; i++) {
			// temperature changing wildly is an indication of a transfer error
			if (fabsf(temperature[i] - temperature_avg) > 1000) {
				perf_count(_bad_transfer_perf);
				return false;
			}
		}

		// use average temperature reading
		const float TEMP_degC = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(TEMP_degC)) {
			_px4_accel.set_temperature(TEMP_degC);
			_px4_gyro.set_temperature(TEMP_degC);
			return true;

		} else {
			perf_count(_bad_transfer_perf);
		}
	}

	return false;
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
	rt_control_state_reset(&_rt_control_state);
	rt_controller_reset();
	// Base time is set once before the periodic callback starts to avoid per-cycle init checks.
	_rt_control_state.t0_us = static_cast<uint64_t>(hrt_absolute_time()) + CONTROL_PERIOD_US;
	_rt_control_state.last_control_ts_us = 0U;
	_rt_control_state.deadline_miss_count = 0U;
	_request_reset = false;
	_fifo_flush_pending = false;
	_latest_publish_seq.store(0);
	_published_seq = 0;
	_pending_telem_seq.store(0);
	_published_telem_seq = 0;
	_telem_publish_count = 0;
	_telem_publish_fail_count = 0;

	hrt_call_every(&_control_loop_call, CONTROL_PERIOD_US, CONTROL_PERIOD_US, ControlLoopTrampoline, this);
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

	// Sensor Read
	const hrt_abstime timestamp_sample = cycle_begin;
	const bool read_ok = ReadSampleDirect(timestamp_sample);
	const uint32_t input_us = static_cast<uint32_t>(hrt_absolute_time() - cycle_begin);

	if (read_ok) {
		if (_failure_count > 0) {
			_failure_count--;
		}

	} else {
		_failure_count++;

		if (_failure_count > 10) {
			_request_reset = true;
		}
	}

	// Compute Output
	float motor_norm[MOTOR_COUNT] {};
	LatestOptiSample opti_sample{};
	(void)CopyLatestOptiSample(opti_sample);
	const rt_control_opti_sample_t opti_input = BuildRtControlOptiSample(cycle_begin, opti_sample);
	const hrt_abstime control_begin = hrt_absolute_time();
	rt_controller(_latest_accel_m_s2, _latest_gyro_rad_s, &opti_input, motor_norm);
	const uint32_t control_us = static_cast<uint32_t>(hrt_absolute_time() - control_begin);

	// Write actuator
	ActuatorWriteResult write_result{};
	const hrt_abstime output_begin = hrt_absolute_time();
	WriteStep(motor_norm, write_result);
	const uint32_t output_us = static_cast<uint32_t>(hrt_absolute_time() - output_begin);

	// Telemetry Write & Send
	TelemetryStep(cycle_begin, input_us, control_us, output_us, motor_norm, write_result, opti_input);
}

bool IIM42652::ReadSampleDirect(const hrt_abstime &timestamp_sample)
{
	DirectDataRegisterBuffer buffer{};
	SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_0);

	if (transfer(reinterpret_cast<uint8_t *>(&buffer), reinterpret_cast<uint8_t *>(&buffer), sizeof(buffer)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}

	const int16_t accel_x = combine(buffer.accel_data_x1, buffer.accel_data_x0);
	const int16_t accel_y = combine(buffer.accel_data_y1, buffer.accel_data_y0);
	const int16_t accel_z = combine(buffer.accel_data_z1, buffer.accel_data_z0);
	const int16_t gyro_x = combine(buffer.gyro_data_x1, buffer.gyro_data_x0);
	const int16_t gyro_y = combine(buffer.gyro_data_y1, buffer.gyro_data_y0);
	const int16_t gyro_z = combine(buffer.gyro_data_z1, buffer.gyro_data_z0);
	const int16_t temp = combine(buffer.temp_data1, buffer.temp_data0);

	const float accel_raw_x = static_cast<float>(accel_x);
	const float accel_raw_y = static_cast<float>((accel_y == INT16_MIN) ? INT16_MAX : -accel_y);
	const float accel_raw_z = static_cast<float>((accel_z == INT16_MIN) ? INT16_MAX : -accel_z);
	const float gyro_raw_x = static_cast<float>(gyro_x);
	const float gyro_raw_y = static_cast<float>((gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y);
	const float gyro_raw_z = static_cast<float>((gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z);

	static constexpr float accel_scale = CONSTANTS_ONE_G / 2048.f; // +/-16g
	static constexpr float gyro_scale = math::radians(2000.f / 32768.f); // +/-2000 dps

	_latest_accel_m_s2[0] = accel_raw_x * accel_scale;
	_latest_accel_m_s2[1] = accel_raw_y * accel_scale;
	_latest_accel_m_s2[2] = accel_raw_z * accel_scale;

	_latest_gyro_rad_s[0] = gyro_raw_x * gyro_scale;
	_latest_gyro_rad_s[1] = gyro_raw_y * gyro_scale;
	_latest_gyro_rad_s[2] = gyro_raw_z * gyro_scale;

	const float temp_degC = (temp / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	const uint32_t seq = _latest_publish_seq.load();
	_latest_publish_seq.store(seq + 1); // writer begin (odd)
	_latest_publish_sample.timestamp_sample = timestamp_sample;
	_latest_publish_sample.accel_raw[0] = accel_raw_x;
	_latest_publish_sample.accel_raw[1] = accel_raw_y;
	_latest_publish_sample.accel_raw[2] = accel_raw_z;
	_latest_publish_sample.gyro_raw[0] = gyro_raw_x;
	_latest_publish_sample.gyro_raw[1] = gyro_raw_y;
	_latest_publish_sample.gyro_raw[2] = gyro_raw_z;
	_latest_publish_sample.temperature_degC = temp_degC;
	_latest_publish_seq.store(seq + 2); // writer end (even)

	return true;
}

void IIM42652::PublishSampleOutsideIRQ()
{
	LatestPublishSample sample{};
	uint32_t seq_begin{0};
	uint32_t seq_end{0};

	do {
		seq_begin = _latest_publish_seq.load();

		if (seq_begin == 0 || (seq_begin & 1u)) {
			return;
		}

		sample = _latest_publish_sample;
		seq_end = _latest_publish_seq.load();

	} while ((seq_begin != seq_end) || (seq_end & 1u));

	if (seq_end == _published_seq) {
		return;
	}

	_published_seq = seq_end;

	if (PX4_ISFINITE(sample.temperature_degC)) {
		_px4_accel.set_temperature(sample.temperature_degC);
		_px4_gyro.set_temperature(sample.temperature_degC);
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

	LatestOptiSample sample{};
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
	_latest_opti_sample_seq.store(seq + 1); // writer begin (odd)
	_latest_opti_sample = sample;
	_latest_opti_sample_seq.store(seq + 2); // writer end (even)
}

bool IIM42652::CopyLatestOptiSample(LatestOptiSample &sample) const
{
	uint32_t seq_begin{0};
	uint32_t seq_end{0};

	do {
		seq_begin = _latest_opti_sample_seq.load();

		if (seq_begin == 0 || (seq_begin & 1u)) {
			return false;
		}

		sample = _latest_opti_sample;
		seq_end = _latest_opti_sample_seq.load();

	} while ((seq_begin != seq_end) || (seq_end & 1u));

	return true;
}

rt_control_opti_sample_t IIM42652::BuildRtControlOptiSample(const hrt_abstime &cycle_begin,
		const LatestOptiSample &sample) const
{
	rt_control_opti_sample_t opti{};

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
	const uint64_t age_us = (sample.timestamp_sample > 0 && now_us >= sample.timestamp_sample)
				? (now_us - sample.timestamp_sample)
				: 0ULL;
	opti.age_us = static_cast<uint32_t>(math::min<uint64_t>(age_us, UINT32_MAX));
	opti.valid = sample.valid && (opti.age_us <= OPTI_TIMEOUT_US);
	return opti;
}

void IIM42652::TelemetryStep(const hrt_abstime &cycle_begin, uint32_t input_us, uint32_t control_us, uint32_t output_us,
			     const float motor_norm[MOTOR_COUNT], const ActuatorWriteResult &write_result,
			     const rt_control_opti_sample_t &opti_sample)
{
	TelemetryFrame f{};
	const uint64_t now_us = static_cast<uint64_t>(cycle_begin);
	_rt_control_state.cycle++;
	f.cycle = _rt_control_state.cycle;
	f.ideal_start_s = ((double)(f.cycle - 1U) * (double)CONTROL_PERIOD_US) * 1e-6;
	f.actual_start_s = ((double)(now_us - _rt_control_state.t0_us)) * 1e-6;

	uint32_t dt_actual_us = 0U;

	if ((_rt_control_state.last_control_ts_us != 0U) && (now_us >= _rt_control_state.last_control_ts_us)) {
		dt_actual_us = static_cast<uint32_t>(now_us - _rt_control_state.last_control_ts_us);
	}

	_rt_control_state.last_control_ts_us = now_us;

	if (dt_actual_us > CONTROL_PERIOD_US) {
		const uint32_t skipped = (dt_actual_us / CONTROL_PERIOD_US) - 1U;
		_rt_control_state.deadline_miss_count += skipped;
	}

	if (dt_actual_us > _rt_control_state.max_loop_dt_us) {
		_rt_control_state.max_loop_dt_us = dt_actual_us;
	}

	f.missed_cycles = _rt_control_state.deadline_miss_count;

	for (uint8_t i = 0; i < 3U; i++) {
		f.accel_m_s2[i] = _latest_accel_m_s2[i];
		f.gyro_rad_s[i] = _latest_gyro_rad_s[i];
	}

	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		f.motor_norm[i] = motor_norm[i];
	}

	for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
		f.pwm_us[i] = write_result.pwm_us[i];
	}

	f.input_us = input_us;
	f.control_us = control_us;
	f.output_us = output_us;

	f.exec_us = static_cast<uint32_t>(hrt_absolute_time() - cycle_begin);
	f.slack_us = (f.exec_us < CONTROL_PERIOD_US) ? (CONTROL_PERIOD_US - f.exec_us) : 0U;

	f.opti_x = (opti_sample.seq > 0U) ? opti_sample.x : NAN;
	f.opti_y = (opti_sample.seq > 0U) ? opti_sample.y : NAN;
	f.opti_z = (opti_sample.seq > 0U) ? opti_sample.z : NAN;
	f.opti_roll = (opti_sample.seq > 0U) ? opti_sample.roll : NAN;
	f.opti_pitch = (opti_sample.seq > 0U) ? opti_sample.pitch : NAN;
	f.opti_yaw = (opti_sample.seq > 0U) ? opti_sample.yaw : NAN;
	f.opti_seq = opti_sample.seq;
	f.opti_age_us = opti_sample.age_us;
	f.opti_valid = opti_sample.valid ? 1U : 0U;

	_last_frame = f;
	QueueTelemetryForPublish(f, cycle_begin);
}

void IIM42652::QueueTelemetryForPublish(const TelemetryFrame &frame, hrt_abstime timestamp)
{
	const uint32_t seq = _pending_telem_seq.load();
	_pending_telem_seq.store(seq + 1); // writer begin (odd)
	_pending_telem.timestamp = timestamp;
	_pending_telem.frame = frame;
	_pending_telem_seq.store(seq + 2); // writer end (even)
}

void IIM42652::PublishTelemetryOutsideIRQ()
{
	if (!_rt_control_telem_pub.advertised()) {
		return;
	}

	PendingTelemetry pending{};
	uint32_t seq_begin{0};
	uint32_t seq_end{0};

	do {
		seq_begin = _pending_telem_seq.load();

		if (seq_begin == 0 || (seq_begin & 1u)) {
			return;
		}

		pending = _pending_telem;
		seq_end = _pending_telem_seq.load();

	} while ((seq_begin != seq_end) || (seq_end & 1u));

	if (seq_end == _published_telem_seq) {
		return;
	}

	_published_telem_seq = seq_end;

	const TelemetryFrame &frame = pending.frame;
	rt_control_telemetry_s msg{};
	msg.timestamp = pending.timestamp;
	msg.cycle = frame.cycle;
	msg.actual_start_s = frame.actual_start_s;
	msg.exec_us = frame.exec_us;
	msg.input_us = frame.input_us;
	msg.control_us = frame.control_us;
	msg.output_us = frame.output_us;
	msg.missed_cycles = frame.missed_cycles;
	msg.opti_x = frame.opti_x;
	msg.opti_y = frame.opti_y;
	msg.opti_z = frame.opti_z;
	msg.opti_roll = frame.opti_roll;
	msg.opti_pitch = frame.opti_pitch;
	msg.opti_yaw = frame.opti_yaw;
	msg.opti_seq = frame.opti_seq;
	msg.opti_age_us = frame.opti_age_us;
	msg.opti_valid = frame.opti_valid;

	for (int i = 0; i < 3; ++i) {
		msg.accel_m_s2[i] = frame.accel_m_s2[i];
		msg.gyro_rad_s[i] = frame.gyro_rad_s[i];
	}

	for (int i = 0; i < MOTOR_COUNT; ++i) {
		msg.motor_norm[i] = frame.motor_norm[i];
		msg.pwm_us[i] = frame.pwm_us[i];
	}

	if (_rt_control_telem_pub.publish(msg)) {
		_telem_publish_count++;

	} else {
		_telem_publish_fail_count++;
	}
}

void IIM42652::WriteStep(const float motor_norm[MOTOR_COUNT], ActuatorWriteResult &out)
{
	(void)motor_norm;

	const MotorOutputMode output_mode = static_cast<MotorOutputMode>(_motor_output_mode.load());
	uint16_t pwm = PWM_MIN_US;

	if (output_mode == MotorOutputMode::EscCalHigh) {
		pwm = PWM_MAX_US;

	} else if (output_mode == MotorOutputMode::EscTest) {
		pwm = _esc_test_pwm_us.load();
	}

	for (int i = 0; i < MOTOR_COUNT; i++) {
		out.pwm_us[i] = pwm;
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

	// Preload minimum-throttle values before enabling the outputs, then
	// assert them once more immediately after arm to reduce startup transients.
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
