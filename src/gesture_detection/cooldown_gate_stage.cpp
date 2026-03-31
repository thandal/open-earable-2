#include "cooldown_gate_stage.h"

#include <errno.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(cooldown_gate_stage, LOG_LEVEL_DBG);

CooldownGateStage::CooldownGateStage(uint32_t cooldown_us)
	: SensorProcessingStage(1),
	  cooldown_us(cooldown_us),
	  have_last_emit(false),
	  last_emit_timestamp_us(0U)
{
}

int CooldownGateStage::process(const struct sensor_data *const input[], struct sensor_data *output)
{
	if (input == nullptr || input[0] == nullptr || output == nullptr) {
		return -EINVAL;
	}

	const uint64_t now_us = input[0]->time;
	if (have_last_emit && now_us < last_emit_timestamp_us + cooldown_us) {
		LOG_INF("Cooldown reject at %llu us: delta=%llu us cooldown=%u us",
			(unsigned long long)now_us,
			(unsigned long long)(now_us - last_emit_timestamp_us), cooldown_us);
		return 1;
	}

	last_emit_timestamp_us = now_us;
	have_last_emit = true;
	LOG_INF("Cooldown pass at %llu us", (unsigned long long)now_us);
	*output = *input[0];
	return 0;
}
