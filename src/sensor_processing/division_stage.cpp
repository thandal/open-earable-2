#include "division_stage.h"

#include <errno.h>

#include "processing_utils.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(division_stage, LOG_LEVEL_INF);

DivisionStage::DivisionStage(size_t in_ports, enum ParseType input_type, bool sync_channels, uint64_t threshold_us)
    : SensorProcessingStage(in_ports), parse_type(input_type), sync_channels(sync_channels), threshold_us(threshold_us) {
}

DivisionStage::~DivisionStage() {
}

template<typename T>
int divide_values(const struct sensor_data *const input[], size_t in_ports, double *result) {
    double quotient = static_cast<double>(decode_sensor_data<T>(*input[0]));
    for (size_t i = 1; i < in_ports; i++) {
        double divisor = static_cast<double>(decode_sensor_data<T>(*input[i]));
        if (divisor == 0.0) {
            return -EDOM;
        }
        LOG_DBG("DivisionStage: dividing %.2f by %.2f", quotient, divisor);
        quotient /= divisor;
    }

    *result = quotient;
    return 0;
}

int DivisionStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input == nullptr || input[0] == nullptr || output == nullptr) {
        return -EINVAL;
    }

    if (this->sync_channels) {
        if (!check_timestamp_sync(input, this->get_in_ports(), this->threshold_us)) {
            return -1;
        }
    }

    *output = *(input[0]);

    switch (this->parse_type) {
        case PARSE_TYPE_UINT8: {
            double result;
            if (divide_values<uint8_t>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT8: {
            double result;
            if (divide_values<int8_t>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_UINT16: {
            double result;
            if (divide_values<uint16_t>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT16: {
            double result;
            if (divide_values<int16_t>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_UINT32: {
            double result;
            if (divide_values<uint32_t>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_INT32: {
            double result;
            if (divide_values<int32_t>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_FLOAT: {
            double result;
            if (divide_values<float>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double result;
            if (divide_values<double>(input, this->get_in_ports(), &result) != 0) return -EDOM;
            output->size = sizeof(result);
            memcpy(output->data, &result, sizeof(result));
            break;
        }
        default:
            return -2;
    }

    return 0;
}
