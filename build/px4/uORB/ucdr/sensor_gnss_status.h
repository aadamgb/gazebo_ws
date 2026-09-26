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
#include <uORB/topics/sensor_gnss_status.h>


static inline constexpr int ucdr_topic_size_sensor_gnss_status()
{
	return 17;
}

static inline bool ucdr_serialize_sensor_gnss_status(const void* data, ucdrBuffer& buf, int64_t time_offset = 0)
{
	const sensor_gnss_status_s& topic = *static_cast<const sensor_gnss_status_s*>(data);
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.device_id) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.device_id, sizeof(topic.device_id));
	buf.iterator += sizeof(topic.device_id);
	buf.offset += sizeof(topic.device_id);
	static_assert(sizeof(topic.quality_available) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.quality_available, sizeof(topic.quality_available));
	buf.iterator += sizeof(topic.quality_available);
	buf.offset += sizeof(topic.quality_available);
	static_assert(sizeof(topic.quality_corrections) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.quality_corrections, sizeof(topic.quality_corrections));
	buf.iterator += sizeof(topic.quality_corrections);
	buf.offset += sizeof(topic.quality_corrections);
	static_assert(sizeof(topic.quality_receiver) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.quality_receiver, sizeof(topic.quality_receiver));
	buf.iterator += sizeof(topic.quality_receiver);
	buf.offset += sizeof(topic.quality_receiver);
	static_assert(sizeof(topic.quality_gnss_signals) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.quality_gnss_signals, sizeof(topic.quality_gnss_signals));
	buf.iterator += sizeof(topic.quality_gnss_signals);
	buf.offset += sizeof(topic.quality_gnss_signals);
	static_assert(sizeof(topic.quality_post_processing) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.quality_post_processing, sizeof(topic.quality_post_processing));
	buf.iterator += sizeof(topic.quality_post_processing);
	buf.offset += sizeof(topic.quality_post_processing);
	return true;
}

static inline bool ucdr_deserialize_sensor_gnss_status(ucdrBuffer& buf, sensor_gnss_status_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.device_id) == 4, "size mismatch");
	memcpy(&topic.device_id, buf.iterator, sizeof(topic.device_id));
	buf.iterator += sizeof(topic.device_id);
	buf.offset += sizeof(topic.device_id);
	static_assert(sizeof(topic.quality_available) == 1, "size mismatch");
	memcpy(&topic.quality_available, buf.iterator, sizeof(topic.quality_available));
	buf.iterator += sizeof(topic.quality_available);
	buf.offset += sizeof(topic.quality_available);
	static_assert(sizeof(topic.quality_corrections) == 1, "size mismatch");
	memcpy(&topic.quality_corrections, buf.iterator, sizeof(topic.quality_corrections));
	buf.iterator += sizeof(topic.quality_corrections);
	buf.offset += sizeof(topic.quality_corrections);
	static_assert(sizeof(topic.quality_receiver) == 1, "size mismatch");
	memcpy(&topic.quality_receiver, buf.iterator, sizeof(topic.quality_receiver));
	buf.iterator += sizeof(topic.quality_receiver);
	buf.offset += sizeof(topic.quality_receiver);
	static_assert(sizeof(topic.quality_gnss_signals) == 1, "size mismatch");
	memcpy(&topic.quality_gnss_signals, buf.iterator, sizeof(topic.quality_gnss_signals));
	buf.iterator += sizeof(topic.quality_gnss_signals);
	buf.offset += sizeof(topic.quality_gnss_signals);
	static_assert(sizeof(topic.quality_post_processing) == 1, "size mismatch");
	memcpy(&topic.quality_post_processing, buf.iterator, sizeof(topic.quality_post_processing));
	buf.iterator += sizeof(topic.quality_post_processing);
	buf.offset += sizeof(topic.quality_post_processing);
	return true;
}
