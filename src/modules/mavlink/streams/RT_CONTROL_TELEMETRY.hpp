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
		uint64_t timestamp_us{0};
		uint32_t loop_dt_us{0};
		uint32_t exec_us{0};
		float accel_m_s2[3]{};
		float gyro_rad_s[3]{};
		uint16_t motor_pwm_us[4]{}; // [us]
		float vision_pos_m[3]{}; // [m]
		float vision_rpy_rad[3]{}; // [rad]
		uint32_t vision_age_us{0}; // [us]
		uint8_t vision_valid{0};
		uint8_t padding[3]{};
	};

	static_assert(sizeof(RtControlTunnelPayload) == 80, "Unexpected RT control tunnel payload size");
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
		payload.timestamp_us = telemetry.timestamp;
		payload.loop_dt_us = telemetry.loop_dt_us;
		payload.exec_us = telemetry.exec_us;
		memcpy(payload.accel_m_s2, telemetry.accel_m_s2, sizeof(payload.accel_m_s2));
		memcpy(payload.gyro_rad_s, telemetry.gyro_rad_s, sizeof(payload.gyro_rad_s));
		memcpy(payload.motor_pwm_us, telemetry.motor_pwm_us, sizeof(payload.motor_pwm_us));
		memcpy(payload.vision_pos_m, telemetry.vision_pos_m, sizeof(payload.vision_pos_m));
		memcpy(payload.vision_rpy_rad, telemetry.vision_rpy_rad, sizeof(payload.vision_rpy_rad));
		payload.vision_age_us = telemetry.vision_age_us;
		payload.vision_valid = telemetry.vision_valid;

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
