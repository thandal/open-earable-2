#include "multiply_stage.h"

#include "processing_utils.h"

MultiplyStage::MultiplyStage(size_t in_ports, enum ParseType input_type, bool sync_channels, uint64_t threshold_us)
    : SensorProcessingStage(in_ports), parse_type(input_type), sync_channels(sync_channels), threshold_us(threshold_us) {
}

MultiplyStage::~MultiplyStage() {
}

int MultiplyStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (this->sync_channels) {
        if (!check_timestamp_sync(input, this->get_in_ports(), this->threshold_us)) {
            return -1;
        }
    }

    *output = *(input[0]);

    switch (this->parse_type) {
        case PARSE_TYPE_UINT8: {
            uint8_t product = 1;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                uint8_t val = decode_sensor_data<uint8_t>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_INT8: {
            int8_t product = 1;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                int8_t val = decode_sensor_data<int8_t>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_UINT16: {
            uint16_t product = 1;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                uint16_t val = decode_sensor_data<uint16_t>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_INT16: {
            int16_t product = 1;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                int16_t val = decode_sensor_data<int16_t>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t product = 1;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                uint32_t val = decode_sensor_data<uint32_t>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_INT32: {
            int32_t product = 1;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                int32_t val = decode_sensor_data<int32_t>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_FLOAT: {
            float product = 1.0f;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                float val = decode_sensor_data<float>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double product = 1.0;
            for (size_t i = 0; i < this->get_in_ports(); i++) {
                double val = decode_sensor_data<double>(*input[i]);
                product *= val;
            }
            output->size = sizeof(product);
            memcpy(output->data, &product, sizeof(product));
            break;
        }
        default:
            return -2; // Unsupported type
    }
    return 0;
}
