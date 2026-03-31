#ifndef _RANGE_GATE_STAGE_H_
#define _RANGE_GATE_STAGE_H_

#include "ParseType.h"
#include "sensor_processing_stage.h"

class RangeGateStage : public SensorProcessingStage {
public:
	RangeGateStage(enum ParseType input_type, float low_threshold, float high_threshold);
	~RangeGateStage() override = default;

	int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
	enum ParseType parse_type;
	float low_threshold;
	float high_threshold;
};

#endif /* _RANGE_GATE_STAGE_H_ */
