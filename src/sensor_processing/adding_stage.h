#ifndef ADDING_STAGE_H
#define ADDING_STAGE_H

#include "sensor_processing_stage.h"
#include "ParseType.h"

class AddingStage : public SensorProcessingStage {
public:
    AddingStage(size_t in_ports, enum ParseType input_type, bool sync_channels = false, uint64_t threshold_us = 0);
    ~AddingStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;
    bool sync_channels;
    uint64_t threshold_us;
};

#endif // ADDING_STAGE_H