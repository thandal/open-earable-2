#ifndef _IF_STAGE_H
#define _IF_STAGE_H

#include "sensor_processing_stage.h"

class IfStage : public SensorProcessingStage {
public:
    IfStage(bool sync_channels = false, uint64_t threshold_us = 0);
    virtual ~IfStage() = default;

    virtual int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    bool sync_channels;
    uint64_t threshold_us;
};

#endif // _IF_STAGE_H