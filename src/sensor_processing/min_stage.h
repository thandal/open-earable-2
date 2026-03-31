#ifndef _MIN_STAGE_H
#define _MIN_STAGE_H

#include <deque>

#include "ParseType.h"
#include "sensor_processing_stage.h"

class MinStage : public SensorProcessingStage {
public:
    MinStage(enum ParseType input_type, size_t window_length);
    ~MinStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

    void set_window_length(size_t window_length);
    size_t get_window_length() const;

private:
    enum ParseType parse_type;
    size_t window_length;
    std::deque<float> samples;
};

#endif // _MIN_STAGE_H
