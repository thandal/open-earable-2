#ifndef _INTEGRAL_STAGE_H
#define _INTEGRAL_STAGE_H

#include <deque>

#include "ParseType.h"
#include "sensor_processing_stage.h"

class IntegralStage : public SensorProcessingStage {
public:
    IntegralStage(enum ParseType input_type, size_t window_length);
    ~IntegralStage();

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

    void set_window_length(size_t window_length);
    size_t get_window_length() const;

private:
    struct TimedSample {
        float value;
        uint64_t time_us;
    };

    float trapezoid_area(const TimedSample& left, const TimedSample& right) const;

    enum ParseType parse_type;
    size_t window_length;
    std::deque<TimedSample> samples;
    float rolling_integral = 0.0f;
};

#endif // _INTEGRAL_STAGE_H
