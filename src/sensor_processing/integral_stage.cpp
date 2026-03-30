#include "integral_stage.h"

#include <errno.h>
#include <cstring>

#include "processing_utils.h"

IntegralStage::IntegralStage(enum ParseType input_type, size_t window_length)
    : SensorProcessingStage(1), parse_type(input_type), window_length(window_length == 0 ? 1 : window_length) {
}

IntegralStage::~IntegralStage() {
}

int IntegralStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input == nullptr || input[0] == nullptr || output == nullptr) {
        return -EINVAL;
    }

    const TimedSample sample = {
        .value = decode_as_float(parse_type, input[0]->data),
        .time_us = input[0]->time,
    };

    if (!samples.empty()) {
        rolling_integral += trapezoid_area(samples.back(), sample);
    }

    samples.push_back(sample);

    while (samples.size() > window_length) {
        if (samples.size() >= 2) {
            rolling_integral -= trapezoid_area(samples[0], samples[1]);
        }
        samples.pop_front();
    }

    *output = *input[0];
    output->size = sizeof(rolling_integral);
    std::memcpy(output->data, &rolling_integral, sizeof(rolling_integral));

    return 0;
}

void IntegralStage::set_window_length(size_t new_window_length) {
    window_length = new_window_length == 0 ? 1 : new_window_length;
    while (samples.size() > window_length) {
        if (samples.size() >= 2) {
            rolling_integral -= trapezoid_area(samples[0], samples[1]);
        }
        samples.pop_front();
    }
}

size_t IntegralStage::get_window_length() const {
    return window_length;
}

float IntegralStage::trapezoid_area(const TimedSample& left, const TimedSample& right) const {
    if (right.time_us <= left.time_us) {
        return 0.0f;
    }

    const float dt_s = static_cast<float>(right.time_us - left.time_us) * 1e-6f;
    return 0.5f * (left.value + right.value) * dt_s;
}
