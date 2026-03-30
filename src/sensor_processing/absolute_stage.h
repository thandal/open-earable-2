#ifndef _ABSOLUTE_STAGE_H
#define _ABSOLUTE_STAGE_H

#include "ParseType.h"
#include "sensor_processing_stage.h"

class AbsoluteStage : public SensorProcessingStage {
public:
    explicit AbsoluteStage(enum ParseType input_type);
    ~AbsoluteStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;
};

#endif // _ABSOLUTE_STAGE_H
