#ifndef _ZERO_CROSSING_DETECTOR_STAGE_H
#define _ZERO_CROSSING_DETECTOR_STAGE_H

#include "sensor_processing_stage.h"
#include "ParseType.h"

class ZeroCrossingDetectorStage : public SensorProcessingStage {
public:
    explicit ZeroCrossingDetectorStage(enum ParseType parse_type);

    int process(const struct sensor_data *const inputs[], struct sensor_data *output) override;

private:
    enum ParseType parse_type;
    bool is_initialized;
    float last_scalar;   // previous sample as a scalar (no borrowed pointers)
};

#endif // _ZERO_CROSSING_DETECTOR_STAGE_H