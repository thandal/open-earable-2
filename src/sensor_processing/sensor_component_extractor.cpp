#include "sensor_component_extractor.h"

#include <cstring>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sensor_component_extractor, LOG_LEVEL_DBG);

SensorComponentExtractor::SensorComponentExtractor(size_t offset, enum ParseType parse_type)
    : SensorProcessingStage(1), offset(offset), parse_type(parse_type) {
}

int SensorComponentExtractor::process(const struct sensor_data *const inputs[],
                                      struct sensor_data *output) {
    const struct sensor_data *in = inputs[0];

    // Copy metadata first (id, timestamp, etc.)
    *output = *in;

    const size_t elem_sz = parseTypeSizes[this->parse_type];

    // Copy the requested component safely (unaligned-safe)
    memcpy(output->data, in->data + this->offset, elem_sz);
    output->size = elem_sz;

    return 0;
}
