#include "constant_source_stage.h"

#include <cstring>
#include <errno.h>

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, uint8_t value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_UINT8, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, int8_t value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_INT8, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, uint16_t value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_UINT16, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, int16_t value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_INT16, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, uint32_t value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_UINT32, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, int32_t value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_INT32, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, float value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_FLOAT, value);
}

ConstantSourceStage::ConstantSourceStage(uint8_t output_id, double value)
    : SensorProcessingStage(0), output_id(output_id) {
    store_constant(PARSE_TYPE_DOUBLE, value);
}

int ConstantSourceStage::process(const struct sensor_data *const input[], struct sensor_data *output) {
    ARG_UNUSED(input);

    if (output == nullptr || constant_size == 0) {
        return -EINVAL;
    }

    output->id = output_id;
    output->size = constant_size;
    output->time = micros();
    std::memcpy(output->data, constant_data, constant_size);

    return 0;
}

bool ConstantSourceStage::is_autonomous_source() const {
    return true;
}

template <typename T>
void ConstantSourceStage::store_constant(enum ParseType type, const T& value) {
    constant_size = parseTypeSizes[type];
    std::memcpy(constant_data, &value, constant_size);
}

template void ConstantSourceStage::store_constant<uint8_t>(enum ParseType type, const uint8_t& value);
template void ConstantSourceStage::store_constant<int8_t>(enum ParseType type, const int8_t& value);
template void ConstantSourceStage::store_constant<uint16_t>(enum ParseType type, const uint16_t& value);
template void ConstantSourceStage::store_constant<int16_t>(enum ParseType type, const int16_t& value);
template void ConstantSourceStage::store_constant<uint32_t>(enum ParseType type, const uint32_t& value);
template void ConstantSourceStage::store_constant<int32_t>(enum ParseType type, const int32_t& value);
template void ConstantSourceStage::store_constant<float>(enum ParseType type, const float& value);
template void ConstantSourceStage::store_constant<double>(enum ParseType type, const double& value);
