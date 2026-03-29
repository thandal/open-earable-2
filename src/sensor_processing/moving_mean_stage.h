#ifndef _MOVING_MEAN_STAGE_H
#define _MOVING_MEAN_STAGE_H

#include <vector>

#include "ParseType.h"
#include "sensor_processing_stage.h"

class MovingMeanStage : public SensorProcessingStage {
public:
    MovingMeanStage(enum ParseType input_type, size_t window_size);
    ~MovingMeanStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;
    std::vector<float> window;
    size_t next_index = 0;
    size_t sample_count = 0;
    float running_sum = 0.0f;
};

#endif // _MOVING_MEAN_STAGE_H
