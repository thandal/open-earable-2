#include "not_stage.h"
#include "processing_utils.h"

NotStage::NotStage()
    : SensorProcessingStage(1) {}
NotStage::~NotStage() {}

int NotStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    uint8_t in = decode_sensor_data<uint8_t>(*input[0]);
    uint8_t out = in == 0 ? 1 : 0;
    *output = *(input[0]);
    output->size = sizeof(uint8_t);
    memcpy(output->data, &out, sizeof(uint8_t));

    return 0;
}
