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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace
{
int find_command_index(int argc, char *argv[], const char *command)
{
	for (int i = 1; i < argc; ++i) {
		if (!strcmp(argv[i], command)) {
			return i;
		}
	}

	return -1;
}
}

void IIM42652::print_usage()
{
	PRINT_MODULE_USAGE_NAME("iim42652", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("imu");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("zero", "Reset RT controller tilt reference to the current pose");
	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_calib", "Output ESC calibration throttle levels on PWM motor outputs");
	PRINT_MODULE_USAGE_COMMAND_DESCR("esc_test", "Apply a capped low-throttle PWM test on PWM motor outputs");
	PRINT_MODULE_USAGE_ARG("high|low|status", "ESC calibration command", true);
	PRINT_MODULE_USAGE_ARG("<0..10>", "ESC low-throttle test percent", true);
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(false, true);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_PARAM_INT('C', 0, 0, 35000, "Input clock frequency (Hz)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

extern "C" int iim42652_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = IIM42652;
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = SPI_SPEED;

	while ((ch = cli.getOpt(argc, argv, "C:R:")) != EOF) {
		switch (ch) {
		case 'C':
			cli.custom1 = atoi(cli.optArg());
			break;

		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_IMU_DEVTYPE_IIM42652);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	if (!strcmp(verb, "zero")) {
		if (iterator.runningInstancesCount() == 0) {
			PX4_ERR("driver not running");
			return -1;
		}

		cli.custom1 = ThisDriver::CLI_CUSTOM_RT_ZERO;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	if (!strcmp(verb, "esc_calib")) {
		if (iterator.runningInstancesCount() == 0) {
			PX4_ERR("driver not running");
			return -1;
		}

		const int verb_index = find_command_index(argc, argv, verb);

		if (verb_index < 0 || verb_index + 1 >= argc) {
			PX4_ERR("missing esc_calib subcommand");
			ThisDriver::print_usage();
			return -1;
		}

		const char *subcommand = argv[verb_index + 1];

		if (!strcmp(subcommand, "high")) {
			cli.custom1 = ThisDriver::CLI_CUSTOM_ESC_CAL_HIGH;
			return ThisDriver::module_custom_method(cli, iterator);
		}

		if (!strcmp(subcommand, "low")) {
			cli.custom1 = ThisDriver::CLI_CUSTOM_ESC_CAL_LOW;
			return ThisDriver::module_custom_method(cli, iterator);
		}

		if (!strcmp(subcommand, "status")) {
			cli.custom1 = ThisDriver::CLI_CUSTOM_ESC_CAL_STATUS;
			return ThisDriver::module_custom_method(cli, iterator, false);
		}

		PX4_ERR("unknown esc_calib subcommand");
		ThisDriver::print_usage();
		return -1;
	}

	if (!strcmp(verb, "esc_test")) {
		if (iterator.runningInstancesCount() == 0) {
			PX4_ERR("driver not running");
			return -1;
		}

		const int verb_index = find_command_index(argc, argv, verb);

		if (verb_index < 0 || verb_index + 1 >= argc) {
			PX4_ERR("missing esc_test percent");
			ThisDriver::print_usage();
			return -1;
		}

		char *endptr = nullptr;
		const long percent = strtol(argv[verb_index + 1], &endptr, 10);

		if ((endptr == nullptr) || (*endptr != '\0')) {
			PX4_ERR("invalid esc_test percent");
			return -1;
		}

		if ((percent < 0) || (percent > ThisDriver::ESC_TEST_MAX_PERCENT)) {
			PX4_ERR("esc_test percent must be between 0 and %u", (unsigned)ThisDriver::ESC_TEST_MAX_PERCENT);
			return -1;
		}

		cli.custom1 = ThisDriver::CLI_CUSTOM_ESC_TEST;
		cli.custom2 = (int)percent;
		return ThisDriver::module_custom_method(cli, iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
