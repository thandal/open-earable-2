#include "subtraction_stage.h"

#include "processing_utils.h"

SubtractionStage::SubtractionStage(size_t in_ports, enum ParseType input_type, bool sync_channels, uint64_t threshold_us)
    : SensorProcessingStage(in_ports), parse_type(input_type), sync_channels(sync_channels), threshold_us(threshold_us) {
}

SubtractionStage::~SubtractionStage() {
}

template<typename T>
T subtract_values(const struct sensor_data *const input[], size_t in_ports) {
    T result = decode_sensor_data<T>(*input[0]);
    for (size_t i = 1; i < in_ports; i++) {
        result -= decode_sensor_data<T>(*input[i]);
    }
    return result;
}

int SubtractionStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (this->sync_channels) {
        if (!check_timestamp_sync(input, this->get_in_ports(), this->threshold_us)) {
            return -1;
        }
    }

    *output = *(input[0]);

    switch (this->parse_type) {
        case PARSE_TYPE_UINT8: {
            uint8_t result = subtract_values<uint8_t>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT8: {
            int8_t result = subtract_values<int8_t>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_UINT16: {
            uint16_t result = subtract_values<uint16_t>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT16: {
            int16_t result = subtract_values<int16_t>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t result = subtract_values<uint32_t>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT32: {
            int32_t result = subtract_values<int32_t>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_FLOAT: {
            float result = subtract_values<float>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double result = subtract_values<double>(input, this->get_in_ports());
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        default:
            return -2;
    }

    return 0;
}
