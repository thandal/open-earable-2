#ifndef _DERIVATIVE_STAGE_H
#define _DERIVATIVE_STAGE_H

#include <deque>

#include "ParseType.h"
#include "sensor_processing_stage.h"

class DerivativeStage : public SensorProcessingStage {
public:
    DerivativeStage(enum ParseType input_type, size_t window_length);
    ~DerivativeStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

    void set_window_length(size_t window_length);
    size_t get_window_length() const;

private:
    struct TimedSample {
        float value;
        uint64_t time_us;
    };

    enum ParseType parse_type;
    size_t window_length;
    std::deque<TimedSample> samples;
};

#endif // _DERIVATIVE_STAGE_H
