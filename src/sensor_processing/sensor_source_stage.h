#ifndef _SENSOR_SOURCE_STAGE_H
#define _SENSOR_SOURCE_STAGE_H

#include "sensor_processing_stage.h"

class SensorSourceStage : public SensorProcessingStage {
public:
    SensorSourceStage(uint8_t sensor_id);

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

    uint8_t get_sensor_id() const;

private:
    uint8_t sensor_id;
};

#endif // _SENSOR_SOURCE_STAGE_H