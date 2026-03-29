#ifndef _CONSTANT_SOURCE_STAGE_H
#define _CONSTANT_SOURCE_STAGE_H

#include <cstdint>

#include "ParseType.h"
#include "sensor_processing_stage.h"

class ConstantSourceStage : public SensorProcessingStage {
public:
    ConstantSourceStage(uint8_t output_id, uint8_t value);
    ConstantSourceStage(uint8_t output_id, int8_t value);
    ConstantSourceStage(uint8_t output_id, uint16_t value);
    ConstantSourceStage(uint8_t output_id, int16_t value);
    ConstantSourceStage(uint8_t output_id, uint32_t value);
    ConstantSourceStage(uint8_t output_id, int32_t value);
    ConstantSourceStage(uint8_t output_id, float value);
    ConstantSourceStage(uint8_t output_id, double value);

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;
    bool is_autonomous_source() const override;

private:
    template <typename T>
    void store_constant(enum ParseType type, const T& value);

    uint8_t output_id;
    uint8_t constant_data[sizeof(double)] = {};
    uint8_t constant_size = 0;
};

#endif // _CONSTANT_SOURCE_STAGE_H
