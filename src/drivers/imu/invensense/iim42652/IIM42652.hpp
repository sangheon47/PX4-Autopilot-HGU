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

/**
 * @file IIM42652.hpp
 *
 * Driver for the Invensense IIM42652 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_IIM42652_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <stddef.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/rt_control_telemetry.h>
#include <uORB/topics/vehicle_odometry.h>

#include "include/control_main.h"
#include "include/control_telemetry.h"

using namespace InvenSense_IIM42652;

class IIM42652 : public device::SPI, public I2CSPIDriver<IIM42652>
{
public:
	static constexpr uint8_t CLI_CUSTOM_ESC_CAL_HIGH{1};
	static constexpr uint8_t CLI_CUSTOM_ESC_CAL_LOW{2};
	static constexpr uint8_t CLI_CUSTOM_ESC_CAL_STATUS{3};
	static constexpr uint8_t CLI_CUSTOM_ESC_TEST{4};
	static constexpr uint8_t CLI_CUSTOM_RT_AUTO{5};
	static constexpr uint8_t ESC_TEST_MAX_PERCENT{10};

	IIM42652(const I2CSPIDriverConfig &config);
	~IIM42652() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	void custom_method(const BusCLIArguments &cli) override;
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr float FIFO_SAMPLE_DT{1e6f / 1000.f};     // 1000 Hz accel & gyro ODR configured
	static constexpr float GYRO_RATE{1e6f / FIFO_SAMPLE_DT};
	static constexpr float ACCEL_RATE{1e6f / FIFO_SAMPLE_DT};

	static constexpr float FIFO_TIMESTAMP_SCALING{16.f *(32.f / 30.f)}; // Used when not using clock input

	// maximum FIFO samples per transfer is limited to the size of sensor_accel_fifo/sensor_gyro_fifo
	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_gyro_fifo_s::x) / sizeof(sensor_gyro_fifo_s::x[0]), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]) * (int)(GYRO_RATE / ACCEL_RATE))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::BANK_0::INT_STATUS) | DIR_READ};
		uint8_t INT_STATUS{0};
		uint8_t FIFO_COUNTH{0};
		uint8_t FIFO_COUNTL{0};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (4 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_bank0_config_t {
		Register::BANK_0 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank1_config_t {
		Register::BANK_1 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	struct register_bank2_config_t {
		Register::BANK_2 reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Reset();

	bool Configure();
	void ConfigureSampleRate(int sample_rate);
	void ConfigureFIFOWatermark(uint8_t samples);
	void ConfigureCLKIN();

	void SelectRegisterBank(enum REG_BANK_SEL_BIT bank, bool force = false);
	void SelectRegisterBank(Register::BANK_0 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_0); }
	void SelectRegisterBank(Register::BANK_1 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_1); }
	void SelectRegisterBank(Register::BANK_2 reg) { SelectRegisterBank(REG_BANK_SEL_BIT::BANK_SEL_2); }

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	template <typename T> bool RegisterCheck(const T &reg_cfg);
	template <typename T> uint8_t RegisterRead(T reg);
	template <typename T> void RegisterWrite(T reg, uint8_t value);
	template <typename T> void RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits);
	template <typename T> void RegisterSetBits(T reg, uint8_t setbits) { RegisterSetAndClearBits(reg, setbits, 0); }
	template <typename T> void RegisterClearBits(T reg, uint8_t clearbits) { RegisterSetAndClearBits(reg, 0, clearbits); }

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples);
	bool ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples);

	const spi_drdy_gpio_t _drdy_gpio;

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	hrt_abstime _reset_timestamp{0};
	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};
	int _failure_count{0};

	bool _enable_clock_input{false};
	float _input_clock_freq{0.f};

	enum REG_BANK_SEL_BIT _last_register_bank {REG_BANK_SEL_BIT::BANK_SEL_0};

	px4::atomic<hrt_abstime> _drdy_timestamp_sample{0};
	bool _data_ready_interrupt_enabled{false};

	enum class STATE : uint8_t {
		RESET,
		WAIT_FOR_RESET,
		CONFIGURE,
		RT_LOOP_INIT,
		RT_LOOP_RUN,
	} _state{STATE::RESET};

	uint16_t _fifo_empty_interval_us{1250}; // default 1250 us / 800 Hz transfer interval
	int32_t _fifo_gyro_samples{static_cast<int32_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};

	uint8_t _checked_register_bank0{0};
	static constexpr uint8_t size_register_bank0_cfg{16};
	register_bank0_config_t _register_bank0_cfg[size_register_bank0_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_0::INT_CONFIG,           INT_CONFIG_BIT::INT1_MODE | INT_CONFIG_BIT::INT1_DRIVE_CIRCUIT, INT_CONFIG_BIT::INT1_POLARITY },
		{ Register::BANK_0::FIFO_CONFIG,          0, FIFO_CONFIG_BIT::FIFO_MODE_STOP_ON_FULL },
		{ Register::BANK_0::INTF_CONFIG1,         INTF_CONFIG1_BIT::AFSR_SET, INTF_CONFIG1_BIT::AFSR_CLEAR}, // RTC_MODE[2] set at runtime
		{ Register::BANK_0::PWR_MGMT0,            PWR_MGMT0_BIT::GYRO_MODE_LOW_NOISE | PWR_MGMT0_BIT::ACCEL_MODE_LOW_NOISE, 0 },
		{ Register::BANK_0::GYRO_CONFIG0,         GYRO_CONFIG0_BIT::GYRO_FS_SEL_2000_DPS | GYRO_CONFIG0_BIT::GYRO_ODR_1KHZ_SET, GYRO_CONFIG0_BIT::GYRO_ODR_1KHZ_CLEAR },
		{ Register::BANK_0::ACCEL_CONFIG0,        ACCEL_CONFIG0_BIT::ACCEL_FS_SEL_16G | ACCEL_CONFIG0_BIT::ACCEL_ODR_1KHZ_SET, ACCEL_CONFIG0_BIT::ACCEL_ODR_1KHZ_CLEAR },
		{ Register::BANK_0::GYRO_CONFIG1,         0, GYRO_CONFIG1_BIT::GYRO_UI_FILT_ORD },
		{ Register::BANK_0::GYRO_ACCEL_CONFIG0,   0, GYRO_ACCEL_CONFIG0_BIT::ACCEL_UI_FILT_BW | GYRO_ACCEL_CONFIG0_BIT::GYRO_UI_FILT_BW },
		{ Register::BANK_0::ACCEL_CONFIG1,        0, ACCEL_CONFIG1_BIT::ACCEL_UI_FILT_ORD },
		{ Register::BANK_0::TMST_CONFIG,          TMST_CONFIG_BIT::TMST_EN | TMST_CONFIG_BIT::TMST_DELTA_EN | TMST_CONFIG_BIT::TMST_TO_REGS_EN | TMST_CONFIG_BIT::TMST_RES, TMST_CONFIG_BIT::TMST_FSYNC_EN },
		{ Register::BANK_0::FIFO_CONFIG1,         0, FIFO_CONFIG1_BIT::FIFO_WM_GT_TH | FIFO_CONFIG1_BIT::FIFO_HIRES_EN | FIFO_CONFIG1_BIT::FIFO_TEMP_EN | FIFO_CONFIG1_BIT::FIFO_GYRO_EN | FIFO_CONFIG1_BIT::FIFO_ACCEL_EN | FIFO_CONFIG1_BIT::FIFO_TMST_FSYNC_EN },
		{ Register::BANK_0::FIFO_CONFIG2,         0, 0 }, // FIFO_WM[7:0] set at runtime
		{ Register::BANK_0::FIFO_CONFIG3,         0, 0 }, // FIFO_WM[11:8] set at runtime
		{ Register::BANK_0::INT_CONFIG0,          0, INT_CONFIG0_BIT::CLEAR_ON_FIFO_READ },
		{ Register::BANK_0::INT_CONFIG1,          0, INT_CONFIG1_BIT::INT_ASYNC_RESET },
		{ Register::BANK_0::INT_SOURCE0,          0, INT_SOURCE0_BIT::FIFO_THS_INT1_EN },
	};

	uint8_t _checked_register_bank1{0};
	static constexpr uint8_t size_register_bank1_cfg{5};
	register_bank1_config_t _register_bank1_cfg[size_register_bank1_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_1::GYRO_CONFIG_STATIC2,  0, GYRO_CONFIG_STATIC2_BIT::GYRO_NF_DIS | GYRO_CONFIG_STATIC2_BIT::GYRO_AAF_DIS },
		{ Register::BANK_1::GYRO_CONFIG_STATIC3,  GYRO_CONFIG_STATIC3_BIT::GYRO_AAF_DELT_585HZ_SET, GYRO_CONFIG_STATIC3_BIT::GYRO_AAF_DELT_585HZ_CLEAR},
		{ Register::BANK_1::GYRO_CONFIG_STATIC4,  GYRO_CONFIG_STATIC4_BIT::GYRO_AAF_DELTSQR_LSB_585HZ_SET, GYRO_CONFIG_STATIC4_BIT::GYRO_AAF_DELTSQR_LSB_585HZ_CLEAR},
		{ Register::BANK_1::GYRO_CONFIG_STATIC5,  GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_BITSHIFT_585HZ_SET | GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_DELTSQR_MSB_585HZ_SET, GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_BITSHIFT_585HZ_CLEAR | GYRO_CONFIG_STATIC5_BIT::GYRO_AAF_DELTSQR_MSB_585HZ_CLEAR},
		{ Register::BANK_1::INTF_CONFIG5,         0, 0 },
	};

	uint8_t _checked_register_bank2{0};
	static constexpr uint8_t size_register_bank2_cfg{3};
	register_bank2_config_t _register_bank2_cfg[size_register_bank2_cfg] {
		// Register                              | Set bits, Clear bits
		{ Register::BANK_2::ACCEL_CONFIG_STATIC2, ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DELT_585HZ_SET, ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DELT_585HZ_CLEAR | ACCEL_CONFIG_STATIC2_BIT::ACCEL_AAF_DIS },
		{ Register::BANK_2::ACCEL_CONFIG_STATIC3, ACCEL_CONFIG_STATIC3_BIT::ACCEL_AAF_DELTSQR_LSB_585HZ_SET, ACCEL_CONFIG_STATIC3_BIT::ACCEL_AAF_DELTSQR_LSB_585HZ_CLEAR },
		{ Register::BANK_2::ACCEL_CONFIG_STATIC4, ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_BITSHIFT_585HZ_SET | ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_DELTSQR_MSB_SET, ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_BITSHIFT_585HZ_CLEAR | ACCEL_CONFIG_STATIC4_BIT::ACCEL_AAF_DELTSQR_MSB_CLEAR },
	};

	// --- 7-nano low-level realtime stack ---
	// These constants define the direct sensor -> controller -> actuator path.

	static constexpr uint8_t MOTOR_COUNT{static_cast<uint8_t>(MOTOR_NUM)};
	static constexpr uint16_t PWM_MIN_US{1000};
	static constexpr uint16_t PWM_MAX_US{2000};
	static constexpr unsigned MOTOR_PWM_RATE{250U}; // MR-X4 ESC input rate margin below 500 Hz limit

	enum class MotorOutputMode : uint8_t {
		SafeIdle = 0,
		EscCalHigh,
		EscCalLow,
		EscTest,
		Auto
	};

	using TelemFrame = telem_frame_t;

	TelemFrame _last_frame{}; // Most recent fast-loop frame for CLI status output.

	// Fast-loop IMU sample cache used by the controller and PX4 publish bridge.
	imu_regs_t _latest_imu{};
	float _latest_accel[3]{};
	float _latest_gyro[3]{};
	struct LatestSample {
		hrt_abstime timestamp_sample{0};
		float accel_raw[3]{};
		float gyro_raw[3]{};
		float temperature{0.f};
	};
	LatestSample _latest_sample{};
	px4::atomic<uint32_t> _latest_publish_seq{0}; // odd: writer in progress, even: stable
	uint32_t _published_seq{0};

	// External navigation/input cache. Visual odometry is first; RC/GPS/baro can follow here.
	struct LatestOpti {
		hrt_abstime timestamp_sample{0};
		float x{0.f};
		float y{0.f};
		float z{0.f};
		float roll{0.f};
		float pitch{0.f};
		float yaw{0.f};
		uint32_t seq{0};
		bool valid{false};
	};
	static constexpr uint32_t OPTI_TIMEOUT_US{200000}; // stale sample cutoff for telemetry valid flag
	uORB::Subscription _vehicle_visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	LatestOpti _latest_opti{};
	px4::atomic<uint32_t> _latest_opti_sample_seq{0}; // odd: writer in progress, even: stable
	uint32_t _opti_sample_counter{0};

	// Direct PWM output state for the low-level actuator path.
	bool _pwm_initialized{false};
	uint32_t _motor_pwm_mask{(1u << MOTOR_COUNT) - 1};
	px4::atomic<uint8_t> _motor_output_mode{static_cast<uint8_t>(MotorOutputMode::Auto)};
	px4::atomic<uint16_t> _esc_test_pwm{PWM_MIN_US};

	// Slow-side telemetry publication health.
	uORB::Publication<rt_control_telemetry_s> _telem_pub{ORB_ID(rt_control_telemetry)};
	uint32_t _telem_pub_ok{0};
	uint32_t _telem_pub_fail{0};

	// Fast-loop scheduler and reset coordination.
	loop_state_t _loop_state{};
	hrt_call _control_loop_call{};
	bool _control_loop_running{false};
	volatile bool _request_reset{false};
	volatile bool _fifo_flush_pending{false};

	struct ActuatorWrite {
		uint16_t pwm[MOTOR_COUNT]{};
	};

	// Direct actuator output. Implemented in IIM42652_actuator.cpp.
	bool InitActuatorDirect();
	void DeinitActuatorDirect();
	void WriteStep(const control_output_t &out_cmd, ActuatorWrite &out);

	// Slow-side publish bridge. Implemented in IIM42652_bridge.cpp.
	void PublishSampleOutsideIRQ();
	void PublishTelemetryOutsideIRQ();
	void TelemetryStep(const hrt_abstime &cycle_begin, uint32_t input, uint32_t control, uint32_t output,
			  const float motor[MOTOR_COUNT], const ActuatorWrite &actuator,
			  const opti_sample_t &opti);

	// External input cache. Implemented in IIM42652_bridge.cpp.
	void UpdateLatestOptiSampleOutsideIRQ();
	bool CopyLatestOptiSample(LatestOpti &sample) const;
	opti_sample_t BuildOptiSample(const hrt_abstime &cycle_begin, const LatestOpti &sample) const;

	// Fast-loop execution. Implemented in IIM42652_rt_loop.cpp.
	static void ControlLoopTrampoline(void *arg);
	void ControlLoopIRQ();
	void StartControlLoopIRQ();
	void StopControlLoopIRQ();
	bool ReadSampleDirect(const hrt_abstime &timestamp_sample);

};
