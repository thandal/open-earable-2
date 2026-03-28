#ifndef _SWITCH_STAGE_H
#define _SWITCH_STAGE_H

#include "sensor_processing_stage.h"
#include "ParseType.h"

class SwitchStage : public SensorProcessingStage {
public:
    SwitchStage(enum ParseType type, float low_thresh, float high_thresh);
    ~SwitchStage() override;

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    uint8_t current_state = 0;

    enum ParseType parse_type;

    const float low_threshold;
    const float high_threshold;
};

#endif // _SWITCH_STAGE_H