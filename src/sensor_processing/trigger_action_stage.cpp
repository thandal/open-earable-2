#include "trigger_action_stage.h"

#include <errno.h>

#include "processing_utils.h"

TriggerActionStage::TriggerActionStage(enum ParseType trigger_type,
                                       TriggerActionCallback callback,
                                       void *user_data)
    : SensorProcessingStage(1), trigger_type(trigger_type), callback(callback), user_data(user_data) {
}

int TriggerActionStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    ARG_UNUSED(output);

    if (input == nullptr || input[0] == nullptr || callback == nullptr) {
        return -EINVAL;
    }

    const float trigger_value = decode_as_float(trigger_type, input[0]->data);
    if (trigger_value == 0.0f) {
        return 1;
    }

    callback(input[0], user_data);
    return 1;
}
