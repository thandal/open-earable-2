#ifndef _MULTIPLY_STAGE_H
#define _MULTIPLY_STAGE_H

#include "sensor_processing_stage.h"
#include "ParseType.h"

class MultiplyStage : public SensorProcessingStage {
public:
    MultiplyStage(size_t in_ports, enum ParseType input_type, bool sync_channels = true, uint64_t threshold_us = 0);
    ~MultiplyStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;
    bool sync_channels;
    uint64_t threshold_us;
};

#endif // _MULTIPLY_STAGE_H