#include "switch_stage.h"
#include <cstring>            // memcpy
#include "processing_utils.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(switch_stage, LOG_LEVEL_WRN);

SwitchStage::SwitchStage(enum ParseType type, float low_thresh, float high_thresh, bool send_changes_only)
    : SensorProcessingStage(1), parse_type(type), low_threshold(low_thresh), high_threshold(high_thresh), send_changes_only(send_changes_only) {}

SwitchStage::~SwitchStage() {}

int SwitchStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (!input) return -EINVAL;
    const struct sensor_data *in = input[0];

    // Decode current sample into a scalar safely (handles unaligned buffers)
    float cur = decode_as_float(parse_type, in->data);

    switch (parse_type) {
        case PARSE_TYPE_INT32: {
            int32_t v;
            std::memcpy(&v, in->data, sizeof(v));
            LOG_DBG("Time: %lld Value: %d", in->time, v);
            break;
        }
        case PARSE_TYPE_UINT32: {
            uint32_t v;
            std::memcpy(&v, in->data, sizeof(v));
            LOG_DBG("Time: %lld Value: %u", in->time, v);
            break;
        }
        case PARSE_TYPE_FLOAT: {
            float v;
            std::memcpy(&v, in->data, sizeof(v));
            LOG_DBG("Time: %lld Value: %f", in->time, v);
            break;
        }
        case PARSE_TYPE_DOUBLE: {
            double v;
            std::memcpy(&v, in->data, sizeof(v));
            LOG_DBG("Time: %lld Value: %f", in->time, v);
            break;
        }
        default:
            LOG_ERR("Unsupported parse type %d", parse_type);
    }

    uint8_t new_state = current_state;
    if (cur < low_threshold) {
        LOG_DBG("Value %f below low threshold %f", cur, low_threshold);
        new_state = 0;
    } else if (cur > high_threshold) {
        LOG_DBG("Value %f above high threshold %f", cur, high_threshold);
        new_state = 1;
    }

    if (!send_changes_only && (new_state == current_state)) {
        return 1; // no state change
    }

    LOG_DBG("Switch state changed: %d -> %d", current_state, new_state);

    current_state = new_state;

    // Emit event: reuse input metadata, payload is uint8 {0, 1}
    *output = *in;
    output->size = sizeof(uint8_t) + in->size;
    std::memcpy(output->data, in->data, in->size);
    std::memcpy(output->data + in->size, &current_state, sizeof(current_state));

    return 0;
}