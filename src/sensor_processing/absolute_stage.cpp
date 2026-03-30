#include "absolute_stage.h"

#include <cstring>
#include <math.h>
#include <type_traits>

#include "processing_utils.h"

AbsoluteStage::AbsoluteStage(enum ParseType input_type)
    : SensorProcessingStage(1), parse_type(input_type) {
}

AbsoluteStage::~AbsoluteStage() {
}

template<typename T>
T abs_value(const struct sensor_data *const input[]) {
    const T value = decode_sensor_data<T>(*input[0]);
    if constexpr (std::is_unsigned_v<T>) {
        return value;
    } else {
        return static_cast<T>(value < 0 ? -value : value);
    }
}

int AbsoluteStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input == nullptr || input[0] == nullptr || output == nullptr) {
        return -1;
    }

    *output = *(input[0]);

    switch (this->parse_type) {
        case PARSE_TYPE_UINT8: {
            uint8_t result = abs_value<uint8_t>(input);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT8: {
            int8_t result = abs_value<int8_t>(input);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_UINT16: {
            uint16_t result = abs_value<uint16_t>(input);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT16: {
            int16_t result = abs_value<int16_t>(input);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t result = abs_value<uint32_t>(input);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT32: {
            int32_t result = abs_value<int32_t>(input);
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_FLOAT: {
            float result = fabsf(decode_sensor_data<float>(*input[0]));
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double result = fabs(decode_sensor_data<double>(*input[0]));
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        default:
            return -2;
    }

    return 0;
}
