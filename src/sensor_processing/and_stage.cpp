#include "and_stage.h"
#include "processing_utils.h"

AndStage::AndStage(size_t in_ports, bool sync_timestamps, uint64_t threshold_us)
    : SensorProcessingStage(in_ports), sync_timestamps(sync_timestamps), threshold_us(threshold_us) {}
AndStage::~AndStage() {}

int AndStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (this->sync_timestamps) {
        if (!check_timestamp_sync(input, this->get_in_ports(), this->threshold_us)) {
            return -1;
        }
    }

    uint8_t out = 0xff;
    for (size_t i = 0; i < this->get_in_ports(); i++) {
        uint8_t in = decode_sensor_data<uint8_t>(*input[i]);
        out &= in;
    }

    *output = *(input[0]);
    output->size = sizeof(uint8_t);
    memcpy(output->data, &out, sizeof(uint8_t));

    return 0;
}
