#ifndef _LOCAL_PEAK_STAGE_H_
#define _LOCAL_PEAK_STAGE_H_

#include "sensor_processing_stage.h"

class LocalPeakStage : public SensorProcessingStage {
public:
	LocalPeakStage();
	~LocalPeakStage() override = default;

	int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
	bool have_prev_prev;
	bool have_prev;
	struct sensor_data prev_prev_sample;
	struct sensor_data prev_sample;
};

#endif /* _LOCAL_PEAK_STAGE_H_ */
