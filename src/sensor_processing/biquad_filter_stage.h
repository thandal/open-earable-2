#ifndef _BIQUAD_FILTER_STAGE_H
#define _BIQUAD_FILTER_STAGE_H

#include "sensor_processing_stage.h"
#include "BiQuadFilter.h"
#include "ParseType.h"

class BiQuadFilterStage : public SensorProcessingStage {
public:
    BiQuadFilterStage(enum ParseType input_type, BiQuadFilter &filter);
    BiQuadFilterStage(enum ParseType input_type, size_t stages, float coefficients[][5]);
    ~BiQuadFilterStage();
    
    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    BiQuadFilter filter;
    enum ParseType parse_type;
};

#endif // _BIQUAD_FILTER_STAGE_H