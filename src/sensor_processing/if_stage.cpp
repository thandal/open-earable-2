#include "if_stage.h"
#include "processing_utils.h"

IfStage::IfStage(bool sync_channels, uint64_t threshold_us)
    : SensorProcessingStage(2), sync_channels(sync_channels), threshold_us(threshold_us) {}

int IfStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    if (input[0] == nullptr || input[1] == nullptr) {
        return -1; // Error: null input
    }

    if (sync_channels) {
        if (!check_timestamp_sync(input, 2, threshold_us)) {
            return -2;
        }
    }
    
    uint8_t exp = decode_sensor_data<uint8_t>(*input[0]);

    if (exp) {
        memcpy(output, input[1], sizeof(struct sensor_data));
        return 0;
    }
    
    return 1;
}
    