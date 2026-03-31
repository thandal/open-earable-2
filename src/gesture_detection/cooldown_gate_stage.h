#ifndef _COOLDOWN_GATE_STAGE_H_
#define _COOLDOWN_GATE_STAGE_H_

#include <stdint.h>

#include "sensor_processing_stage.h"

class CooldownGateStage : public SensorProcessingStage {
public:
	explicit CooldownGateStage(uint32_t cooldown_us);
	~CooldownGateStage() override = default;

	int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
	uint32_t cooldown_us;
	bool have_last_emit;
	uint64_t last_emit_timestamp_us;
};

#endif /* _COOLDOWN_GATE_STAGE_H_ */
