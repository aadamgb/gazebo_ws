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
#include <uORB/topics/neural_control.h>


static inline constexpr int ucdr_topic_size_neural_control()
{
	return 92;
}

static inline bool ucdr_serialize_neural_control(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const neural_control_s& topic = *static_cast<const neural_control_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.observation) == 60, "size mismatch");
	memcpy(buf.iterator, &topic.observation, sizeof(topic.observation));
	buf.iterator += sizeof(topic.observation);
	buf.offset += sizeof(topic.observation);
	static_assert(sizeof(topic.network_output) == 16, "size mismatch");
	memcpy(buf.iterator, &topic.network_output, sizeof(topic.network_output));
	buf.iterator += sizeof(topic.network_output);
	buf.offset += sizeof(topic.network_output);
	static_assert(sizeof(topic.controller_time) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.controller_time, sizeof(topic.controller_time));
	buf.iterator += sizeof(topic.controller_time);
	buf.offset += sizeof(topic.controller_time);
	static_assert(sizeof(topic.inference_time) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.inference_time, sizeof(topic.inference_time));
	buf.iterator += sizeof(topic.inference_time);
	buf.offset += sizeof(topic.inference_time);
	return true;
}

static inline bool ucdr_deserialize_neural_control(ucdrBuffer& buf, neural_control_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.observation) == 60, "size mismatch");
	memcpy(&topic.observation, buf.iterator, sizeof(topic.observation));
	buf.iterator += sizeof(topic.observation);
	buf.offset += sizeof(topic.observation);
	static_assert(sizeof(topic.network_output) == 16, "size mismatch");
	memcpy(&topic.network_output, buf.iterator, sizeof(topic.network_output));
	buf.iterator += sizeof(topic.network_output);
	buf.offset += sizeof(topic.network_output);
	static_assert(sizeof(topic.controller_time) == 4, "size mismatch");
	memcpy(&topic.controller_time, buf.iterator, sizeof(topic.controller_time));
	buf.iterator += sizeof(topic.controller_time);
	buf.offset += sizeof(topic.controller_time);
	static_assert(sizeof(topic.inference_time) == 4, "size mismatch");
	memcpy(&topic.inference_time, buf.iterator, sizeof(topic.inference_time));
	buf.iterator += sizeof(topic.inference_time);
	buf.offset += sizeof(topic.inference_time);
	return true;
}
