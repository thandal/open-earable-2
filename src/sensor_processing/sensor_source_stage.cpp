#include "sensor_source_stage.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_source_stage, LOG_LEVEL_DBG);

SensorSourceStage::SensorSourceStage(uint8_t sensor_id) : SensorProcessingStage(0), sensor_id(sensor_id) {
    // Constructor implementation
}

int SensorSourceStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    LOG_ERR("This method should not be called, as SensorSourceStage is a source stage");
    return -EINVAL;
}

uint8_t SensorSourceStage::get_sensor_id() const {
    return sensor_id;
}
