#include "sink_stage.h"

#include <errno.h>

SinkStage::SinkStage(SinkStageCallback callback, void *user_data)
    : SensorProcessingStage(1), callback(callback), user_data(user_data) {
}

int SinkStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    ARG_UNUSED(output);

    if (input == nullptr || input[0] == nullptr || callback == nullptr) {
        return -EINVAL;
    }

    callback(input[0], user_data);
    return 1;
}
