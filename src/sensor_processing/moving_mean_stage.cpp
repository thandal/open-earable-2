#include "moving_mean_stage.h"

#include <errno.h>
#include <cstring>

#include "processing_utils.h"

MovingMeanStage::MovingMeanStage(enum ParseType input_type, size_t window_size)
    : SensorProcessingStage(1), parse_type(input_type), window(window_size, 0.0f) {
}

MovingMeanStage::~MovingMeanStage() {
}

int MovingMeanStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input == nullptr || input[0] == nullptr || window.empty()) {
        return -EINVAL;
    }

    const float sample = decode_as_float(parse_type, input[0]->data);

    if (sample_count < window.size()) {
        sample_count++;
    } else {
        running_sum -= window[next_index];
    }

    window[next_index] = sample;
    running_sum += sample;
    next_index = (next_index + 1) % window.size();

    const float mean = running_sum / static_cast<float>(sample_count);

    *output = *input[0];
    output->size = sizeof(mean);
    std::memcpy(output->data, &mean, sizeof(mean));

    return 0;
}
