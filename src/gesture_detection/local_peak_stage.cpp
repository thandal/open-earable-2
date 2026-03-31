#include "local_peak_stage.h"

#include <cstring>
#include <errno.h>

#include "ParseType.h"
#include "processing_utils.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(local_peak_stage, LOG_LEVEL_INF);

LocalPeakStage::LocalPeakStage()
	: SensorProcessingStage(1), have_prev_prev(false), have_prev(false)
{
	memset(&prev_prev_sample, 0, sizeof(prev_prev_sample));
	memset(&prev_sample, 0, sizeof(prev_sample));
}

int LocalPeakStage::process(const struct sensor_data *const input[], struct sensor_data *output)
{
	if (input == nullptr || input[0] == nullptr || output == nullptr) {
		return -EINVAL;
	}

	const struct sensor_data *current = input[0];

	if (!have_prev_prev) {
		prev_prev_sample = *current;
		have_prev_prev = true;
		return 1;
	}

	if (!have_prev) {
		prev_sample = *current;
		have_prev = true;
		return 1;
	}

	const float older = decode_as_float(PARSE_TYPE_FLOAT, prev_prev_sample.data);
	const float middle = decode_as_float(PARSE_TYPE_FLOAT, prev_sample.data);
	const float newer = decode_as_float(PARSE_TYPE_FLOAT, current->data);
	const bool is_local_peak = (middle > older) && (middle >= newer);
	const struct sensor_data peak_sample = prev_sample;

	LOG_DBG("Peak window older=%.4f middle=%.4f newer=%.4f peak=%d",
		(double)older, (double)middle, (double)newer, is_local_peak);

	prev_prev_sample = prev_sample;
	prev_sample = *current;

	if (!is_local_peak) {
		return 1;
	}

	LOG_INF("Local peak detected at %llu us with amplitude %.4f",
		(unsigned long long)peak_sample.time, (double)middle);

	*output = peak_sample;
	return 0;
}
