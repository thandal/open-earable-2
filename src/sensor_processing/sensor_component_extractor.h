#ifndef _SENSOR_COMPONENT_EXTRACTOR_H
#define _SENSOR_COMPONENT_EXTRACTOR_H

#include "sensor_processing_stage.h"
#include "ParseType.h"

class SensorComponentExtractor: public SensorProcessingStage {
public:
    SensorComponentExtractor(size_t offset, enum ParseType parse_type);

    int process(const struct sensor_data *const inputs[], struct sensor_data *output) override;

private:
    size_t offset;
    enum ParseType parse_type;
};

#endif