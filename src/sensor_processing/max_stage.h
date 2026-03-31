#ifndef _MAX_STAGE_H
#define _MAX_STAGE_H

#include <deque>

#include "ParseType.h"
#include "sensor_processing_stage.h"

class MaxStage : public SensorProcessingStage {
public:
    MaxStage(enum ParseType input_type, size_t window_length);
    ~MaxStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

    void set_window_length(size_t window_length);
    size_t get_window_length() const;

private:
    enum ParseType parse_type;
    size_t window_length;
    std::deque<float> samples;
};

#endif // _MAX_STAGE_H
