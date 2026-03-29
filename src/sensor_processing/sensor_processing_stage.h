#ifndef _SENSOR_PROCESSING_STAGE_H
#define _SENSOR_PROCESSING_STAGE_H

#include "openearable_common.h"

class SensorProcessingStage {
public:
    SensorProcessingStage(size_t in_ports);
    virtual ~SensorProcessingStage() = default;

    virtual int process(const struct sensor_data *const input[], struct sensor_data *output) = 0;
    virtual bool is_autonomous_source() const;

    size_t get_in_ports() const {
        return in_ports;
    }

private:
    size_t in_ports;
};

#endif // _SENSOR_PROCESSING_STAGE_H
