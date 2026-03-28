#include "biquad_filter_stage.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(biquad_filter_stage, LOG_LEVEL_DBG);

BiQuadFilterStage::BiQuadFilterStage(enum ParseType input_type, BiQuadFilter &filter) : SensorProcessingStage(1) {
    this->filter = filter;
    this->parse_type = input_type;
}

BiQuadFilterStage::BiQuadFilterStage(enum ParseType input_type, size_t stages, float coefficients[][5]) : SensorProcessingStage(1) {
    this->parse_type = input_type;
    this->filter = BiQuadFilter(stages);
    this->filter.set_coefficients(coefficients, stages);
}

BiQuadFilterStage::~BiQuadFilterStage() {
    //TODO: implement
}

int BiQuadFilterStage::process(const struct sensor_data *const input[],
                               struct sensor_data *output) {
    // Copy metadata
    *output = *input[0];
    output->size = sizeof(float);

    float x = 0.0f;

    switch (parse_type) {
        case PARSE_TYPE_UINT8: {
            uint8_t v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        case PARSE_TYPE_INT8: {
            int8_t v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        case PARSE_TYPE_UINT16: {
            uint16_t v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        case PARSE_TYPE_INT16: {
            int16_t v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        case PARSE_TYPE_INT32: {
            int32_t v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        case PARSE_TYPE_FLOAT: {
            float v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = v;
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double v;
            memcpy(&v, input[0]->data, sizeof(v));
            x = static_cast<float>(v);
            break;
        }
        default:
            LOG_ERR("Unsupported parse type %d", parse_type);
            return -EINVAL;
    }

    float y = x;
    filter.apply(&y, 1);

    // Write back safely
    memcpy(output->data, &y, sizeof(float));

    return 0;
}
