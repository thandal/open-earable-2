#include "derivative_stage.h"

#include <errno.h>
#include <cstring>

#include "processing_utils.h"

DerivativeStage::DerivativeStage(enum ParseType input_type, size_t window_length)
    : SensorProcessingStage(1), parse_type(input_type), window_length(window_length == 0 ? 1 : window_length) {
}

DerivativeStage::~DerivativeStage() {
}

int DerivativeStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input == nullptr || input[0] == nullptr || output == nullptr) {
        return -EINVAL;
    }

    const TimedSample sample = {
        .value = decode_as_float(parse_type, input[0]->data),
        .time_us = input[0]->time,
    };

    samples.push_back(sample);
    while (samples.size() > window_length) {
        samples.pop_front();
    }

    float derivative = 0.0f;
    if (samples.size() >= 2) {
        const TimedSample& first = samples.front();
        const TimedSample& last = samples.back();
        if (last.time_us > first.time_us) {
            const uint64_t dt_us = last.time_us - first.time_us;
            derivative = (last.value - first.value) / (static_cast<float>(dt_us) * 1e-6f);
        }
    }

    *output = *input[0];
    output->size = sizeof(derivative);
    std::memcpy(output->data, &derivative, sizeof(derivative));

    return 0;
}

void DerivativeStage::set_window_length(size_t new_window_length) {
    window_length = new_window_length == 0 ? 1 : new_window_length;
    while (samples.size() > window_length) {
        samples.pop_front();
    }
}

size_t DerivativeStage::get_window_length() const {
    return window_length;
}
