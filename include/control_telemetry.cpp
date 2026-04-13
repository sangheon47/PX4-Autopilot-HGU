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

#include "control_telemetry.h"

#include <px4_platform_common/atomic.h>
#include <string.h>

namespace
{
px4::atomic<uint32_t> g_write_index{0};
px4::atomic<uint32_t> g_read_index{0};
px4::atomic<uint32_t> g_drop_count{0};
px4::atomic<uint32_t> g_high_watermark{0};
telem_item_t g_queue[TELEM_QUEUE_SIZE] {};

void update_high_watermark(uint32_t queued)
{
	const uint32_t current = g_high_watermark.load();

	if (queued > current) {
		g_high_watermark.store(queued);
	}
}
}

extern "C" void telem_reset(void)
{
	memset(g_queue, 0, sizeof(g_queue));
	g_write_index.store(0);
	g_read_index.store(0);
	g_drop_count.store(0);
	g_high_watermark.store(0);
}

extern "C" bool telem_push(uint64_t timestamp, const telem_frame_t *frame)
{
	if (frame == nullptr) {
		return false;
	}

	const uint32_t write_index = g_write_index.load();
	const uint32_t read_index = g_read_index.load();

	if ((write_index - read_index) >= TELEM_QUEUE_SIZE) {
		g_drop_count.fetch_add(1);
		return false;
	}

	telem_item_t &slot = g_queue[write_index % TELEM_QUEUE_SIZE];
	slot.timestamp = timestamp;
	slot.frame = *frame;
	g_write_index.store(write_index + 1);

	update_high_watermark((write_index + 1) - read_index);
	return true;
}

extern "C" bool telem_pop(telem_item_t *item)
{
	if (item == nullptr) {
		return false;
	}

	const uint32_t read_index = g_read_index.load();
	const uint32_t write_index = g_write_index.load();

	if (read_index == write_index) {
		return false;
	}

	*item = g_queue[read_index % TELEM_QUEUE_SIZE];
	g_read_index.store(read_index + 1);
	return true;
}

extern "C" void telem_status(telem_queue_status_t *status)
{
	if (status == nullptr) {
		return;
	}

	const uint32_t write_index = g_write_index.load();
	const uint32_t read_index = g_read_index.load();

	status->queued = write_index - read_index;
	status->capacity = TELEM_QUEUE_SIZE;
	status->dropped = g_drop_count.load();
	status->high_watermark = g_high_watermark.load();
}
