#include "adding_stage.h"
#include "processing_utils.h"

AddingStage::AddingStage(size_t in_ports, enum ParseType input_type, bool sync_channels, uint64_t threshold_us)
    : SensorProcessingStage(in_ports), parse_type(input_type), sync_channels(sync_channels), threshold_us(threshold_us) {
}

AddingStage::~AddingStage() {
}

template<typename T>
T add_values(const struct sensor_data *const input[], size_t in_ports) {
    T sum = 0;
    for (size_t i = 0; i < in_ports; i++) {
        sum += decode_sensor_data<T>(*input[i]);
    }
    return sum;
}

bool check_timestamp_sync(const struct sensor_data *const input[], size_t in_ports, uint64_t threshold_us);

int AddingStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (this->sync_channels) {
        if (!check_timestamp_sync(input, this->get_in_ports(), this->threshold_us)) {
            return -1;
        }
    }

    *output = *(input[0]);

    switch (this->parse_type) {
        case PARSE_TYPE_UINT8: {
            uint8_t sum = add_values<uint8_t>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_INT8: {
            int8_t sum = add_values<int8_t>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_UINT16: {
            uint16_t sum = add_values<uint16_t>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_INT16: {
            int16_t sum = add_values<int16_t>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t sum = add_values<uint32_t>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_INT32: {
            int32_t sum = add_values<int32_t>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_FLOAT: {
            float sum = add_values<float>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double sum = add_values<double>(input, this->get_in_ports());
            output->size = sizeof(sum);
            memcpy(output->data, &sum, sizeof(sum));
            break;
        }
        default:
            return -2; // Unsupported type
    }

    return 0;
}
