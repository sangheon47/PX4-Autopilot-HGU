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

#ifndef RT_CONTROL_TELEMETRY_HPP
#define RT_CONTROL_TELEMETRY_HPP

#include <cstring>

#include <uORB/topics/rt_control_telemetry.h>

class MavlinkStreamRtControlTelemetry : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamRtControlTelemetry(mavlink); }

	static constexpr const char *get_name_static() { return "RT_CONTROL_TELEMETRY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_TUNNEL; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _rt_control_telem_sub.advertised() ? MAVLINK_MSG_ID_TUNNEL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	static constexpr uint16_t RT_CONTROL_TUNNEL_PAYLOAD_TYPE{32768};

	struct RtControlTunnelPayload {
		double actual_start_s{0.0};
		uint32_t cycle{0};
		uint32_t exec_us{0};
		uint32_t input_us{0};
		uint32_t control_us{0};
		uint32_t output_us{0};
		uint32_t missed_cycles{0};
		float accel_m_s2[3]{};
		float gyro_rad_s[3]{};
		float motor_norm[6]{};
		float opti_x{0.f};
		float opti_y{0.f};
		float opti_z{0.f};
		float opti_roll{0.f};
		float opti_pitch{0.f};
		float opti_yaw{0.f};
		uint32_t opti_seq{0};
		uint32_t opti_age_us{0};
		uint8_t opti_valid{0};
		uint8_t padding[7]{};
	};

	static_assert(sizeof(RtControlTunnelPayload) == 120, "Unexpected RT control tunnel payload size");
	static_assert(sizeof(RtControlTunnelPayload) <= 128, "RT control tunnel payload exceeds MAVLink TUNNEL limit");

	explicit MavlinkStreamRtControlTelemetry(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _rt_control_telem_sub{ORB_ID(rt_control_telemetry)};

	bool send() override
	{
		rt_control_telemetry_s telemetry{};

		if (!_rt_control_telem_sub.update(&telemetry)) {
			return false;
		}

		RtControlTunnelPayload payload{};
		payload.actual_start_s = telemetry.actual_start_s;
		payload.cycle = telemetry.cycle;
		payload.exec_us = telemetry.exec_us;
		payload.input_us = telemetry.input_us;
		payload.control_us = telemetry.control_us;
		payload.output_us = telemetry.output_us;
		payload.missed_cycles = telemetry.missed_cycles;
		memcpy(payload.accel_m_s2, telemetry.accel_m_s2, sizeof(payload.accel_m_s2));
		memcpy(payload.gyro_rad_s, telemetry.gyro_rad_s, sizeof(payload.gyro_rad_s));
		memcpy(payload.motor_norm, telemetry.motor_norm, sizeof(payload.motor_norm));
		payload.opti_x = telemetry.opti_x;
		payload.opti_y = telemetry.opti_y;
		payload.opti_z = telemetry.opti_z;
		payload.opti_roll = telemetry.opti_roll;
		payload.opti_pitch = telemetry.opti_pitch;
		payload.opti_yaw = telemetry.opti_yaw;
		payload.opti_seq = telemetry.opti_seq;
		payload.opti_age_us = telemetry.opti_age_us;
		payload.opti_valid = telemetry.opti_valid;

		mavlink_tunnel_t msg{};
		msg.target_system = 0;
		msg.target_component = 0;
		msg.payload_type = RT_CONTROL_TUNNEL_PAYLOAD_TYPE;
		msg.payload_length = static_cast<uint8_t>(sizeof(payload));
		memcpy(msg.payload, &payload, sizeof(payload));

		mavlink_msg_tunnel_send_struct(_mavlink->get_channel(), &msg);
		return true;
	}
};

#endif // RT_CONTROL_TELEMETRY_HPP
