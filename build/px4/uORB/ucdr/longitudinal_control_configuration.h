/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
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


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/longitudinal_control_configuration.h>


static inline constexpr int ucdr_topic_size_longitudinal_control_configuration()
{
	return 38;
}

static inline bool ucdr_serialize_longitudinal_control_configuration(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const longitudinal_control_configuration_s& topic = *static_cast<const longitudinal_control_configuration_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.pitch_min) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pitch_min, sizeof(topic.pitch_min));
	buf.iterator += sizeof(topic.pitch_min);
	buf.offset += sizeof(topic.pitch_min);
	static_assert(sizeof(topic.pitch_max) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.pitch_max, sizeof(topic.pitch_max));
	buf.iterator += sizeof(topic.pitch_max);
	buf.offset += sizeof(topic.pitch_max);
	static_assert(sizeof(topic.throttle_min) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.throttle_min, sizeof(topic.throttle_min));
	buf.iterator += sizeof(topic.throttle_min);
	buf.offset += sizeof(topic.throttle_min);
	static_assert(sizeof(topic.throttle_max) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.throttle_max, sizeof(topic.throttle_max));
	buf.iterator += sizeof(topic.throttle_max);
	buf.offset += sizeof(topic.throttle_max);
	static_assert(sizeof(topic.climb_rate_target) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.climb_rate_target, sizeof(topic.climb_rate_target));
	buf.iterator += sizeof(topic.climb_rate_target);
	buf.offset += sizeof(topic.climb_rate_target);
	static_assert(sizeof(topic.sink_rate_target) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.sink_rate_target, sizeof(topic.sink_rate_target));
	buf.iterator += sizeof(topic.sink_rate_target);
	buf.offset += sizeof(topic.sink_rate_target);
	static_assert(sizeof(topic.speed_weight) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.speed_weight, sizeof(topic.speed_weight));
	buf.iterator += sizeof(topic.speed_weight);
	buf.offset += sizeof(topic.speed_weight);
	static_assert(sizeof(topic.enforce_low_height_condition) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.enforce_low_height_condition, sizeof(topic.enforce_low_height_condition));
	buf.iterator += sizeof(topic.enforce_low_height_condition);
	buf.offset += sizeof(topic.enforce_low_height_condition);
	static_assert(sizeof(topic.disable_underspeed_protection) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.disable_underspeed_protection, sizeof(topic.disable_underspeed_protection));
	buf.iterator += sizeof(topic.disable_underspeed_protection);
	buf.offset += sizeof(topic.disable_underspeed_protection);
	return true;
}

static inline bool ucdr_deserialize_longitudinal_control_configuration(ucdrBuffer& buf, longitudinal_control_configuration_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.pitch_min) == 4, "size mismatch");
	memcpy(&topic.pitch_min, buf.iterator, sizeof(topic.pitch_min));
	buf.iterator += sizeof(topic.pitch_min);
	buf.offset += sizeof(topic.pitch_min);
	static_assert(sizeof(topic.pitch_max) == 4, "size mismatch");
	memcpy(&topic.pitch_max, buf.iterator, sizeof(topic.pitch_max));
	buf.iterator += sizeof(topic.pitch_max);
	buf.offset += sizeof(topic.pitch_max);
	static_assert(sizeof(topic.throttle_min) == 4, "size mismatch");
	memcpy(&topic.throttle_min, buf.iterator, sizeof(topic.throttle_min));
	buf.iterator += sizeof(topic.throttle_min);
	buf.offset += sizeof(topic.throttle_min);
	static_assert(sizeof(topic.throttle_max) == 4, "size mismatch");
	memcpy(&topic.throttle_max, buf.iterator, sizeof(topic.throttle_max));
	buf.iterator += sizeof(topic.throttle_max);
	buf.offset += sizeof(topic.throttle_max);
	static_assert(sizeof(topic.climb_rate_target) == 4, "size mismatch");
	memcpy(&topic.climb_rate_target, buf.iterator, sizeof(topic.climb_rate_target));
	buf.iterator += sizeof(topic.climb_rate_target);
	buf.offset += sizeof(topic.climb_rate_target);
	static_assert(sizeof(topic.sink_rate_target) == 4, "size mismatch");
	memcpy(&topic.sink_rate_target, buf.iterator, sizeof(topic.sink_rate_target));
	buf.iterator += sizeof(topic.sink_rate_target);
	buf.offset += sizeof(topic.sink_rate_target);
	static_assert(sizeof(topic.speed_weight) == 4, "size mismatch");
	memcpy(&topic.speed_weight, buf.iterator, sizeof(topic.speed_weight));
	buf.iterator += sizeof(topic.speed_weight);
	buf.offset += sizeof(topic.speed_weight);
	static_assert(sizeof(topic.enforce_low_height_condition) == 1, "size mismatch");
	memcpy(&topic.enforce_low_height_condition, buf.iterator, sizeof(topic.enforce_low_height_condition));
	buf.iterator += sizeof(topic.enforce_low_height_condition);
	buf.offset += sizeof(topic.enforce_low_height_condition);
	static_assert(sizeof(topic.disable_underspeed_protection) == 1, "size mismatch");
	memcpy(&topic.disable_underspeed_protection, buf.iterator, sizeof(topic.disable_underspeed_protection));
	buf.iterator += sizeof(topic.disable_underspeed_protection);
	buf.offset += sizeof(topic.disable_underspeed_protection);
	return true;
}
