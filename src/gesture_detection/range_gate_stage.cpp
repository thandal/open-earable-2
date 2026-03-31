#include "range_gate_stage.h"

#include <errno.h>

#include "processing_utils.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(range_gate_stage, LOG_LEVEL_DBG);

RangeGateStage::RangeGateStage(enum ParseType input_type, float low_threshold, float high_threshold)
	: SensorProcessingStage(1),
	  parse_type(input_type),
	  low_threshold(low_threshold),
	  high_threshold(high_threshold)
{
}

int RangeGateStage::process(const struct sensor_data *const input[], struct sensor_data *output)
{
	if (input == nullptr || input[0] == nullptr || output == nullptr) {
		return -EINVAL;
	}

	const float value = decode_as_float(parse_type, input[0]->data);
	if (value < low_threshold || value > high_threshold) {
		LOG_INF("Threshold reject at %llu us: value=%.4f range=[%.4f, %.4f]",
			(unsigned long long)input[0]->time, (double)value, (double)low_threshold,
			(double)high_threshold);
		return 1;
	}

	LOG_INF("Threshold pass at %llu us: value=%.4f",
		(unsigned long long)input[0]->time, (double)value);

	*output = *input[0];
	return 0;
}
