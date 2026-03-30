#ifndef _DIVISION_STAGE_H
#define _DIVISION_STAGE_H

#include "ParseType.h"
#include "sensor_processing_stage.h"

class DivisionStage : public SensorProcessingStage {
public:
    DivisionStage(size_t in_ports, enum ParseType input_type, bool sync_channels = true, uint64_t threshold_us = 0);
    ~DivisionStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;
    bool sync_channels;
    uint64_t threshold_us;
};

#endif // _DIVISION_STAGE_H
