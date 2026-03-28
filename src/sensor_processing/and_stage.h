#ifndef _AND_STAGE_H
#define _AND_STAGE_H

#include "sensor_processing_stage.h"

class AndStage: public SensorProcessingStage {
public:
    AndStage(size_t in_ports, bool sync_timestamps=false, uint64_t threshold_us = 0);
    ~AndStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    bool sync_timestamps;
    uint64_t threshold_us;
};

#endif // _AND_STAGE_H