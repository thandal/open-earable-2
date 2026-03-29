#include "sensor_processing_stage.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_processing_stage, LOG_LEVEL_WRN);

SensorProcessingStage::SensorProcessingStage(size_t in_ports)
    : in_ports(in_ports) {
}

bool SensorProcessingStage::is_autonomous_source() const {
    return false;
}
