#include "min_stage.h"

#include <algorithm>
#include <cstring>
#include <errno.h>

#include "processing_utils.h"

MinStage::MinStage(enum ParseType input_type, size_t window_length)
    : SensorProcessingStage(1), parse_type(input_type), window_length(window_length == 0 ? 1 : window_length) {
}

MinStage::~MinStage() {
}

static int encode_minmax_result(enum ParseType parse_type, float value, struct sensor_data *output) {
    switch (parse_type) {
        case PARSE_TYPE_UINT8: {
            uint8_t result = static_cast<uint8_t>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        case PARSE_TYPE_INT8: {
            int8_t result = static_cast<int8_t>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        case PARSE_TYPE_UINT16: {
            uint16_t result = static_cast<uint16_t>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        case PARSE_TYPE_INT16: {
            int16_t result = static_cast<int16_t>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t result = static_cast<uint32_t>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        case PARSE_TYPE_INT32: {
            int32_t result = static_cast<int32_t>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        case PARSE_TYPE_FLOAT: {
            output->size = sizeof(value);
            memcpy(output->data, &value, sizeof(value));
            return 0;
        }
        case PARSE_TYPE_DOUBLE: {
            double result = static_cast<double>(value);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            return 0;
        }
        default:
            return -EINVAL;
    }
}

int MinStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input == nullptr || input[0] == nullptr || output == nullptr) {
        return -EINVAL;
    }

    samples.push_back(decode_as_float(parse_type, input[0]->data));
    while (samples.size() > window_length) {
        samples.pop_front();
    }

    *output = *input[0];
    const float result = *std::min_element(samples.begin(), samples.end());
    return encode_minmax_result(parse_type, result, output);
}

void MinStage::set_window_length(size_t new_window_length) {
    window_length = new_window_length == 0 ? 1 : new_window_length;
    while (samples.size() > window_length) {
        samples.pop_front();
    }
}

size_t MinStage::get_window_length() const {
    return window_length;
}
