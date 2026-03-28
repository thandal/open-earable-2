#ifndef _NOT_STAGE_H
#define _NOT_STAGE_H

#include "sensor_processing_stage.h"

class NotStage: public SensorProcessingStage {
public:
    NotStage();
    ~NotStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output);
};

#endif // _NOT_STAGE_H