#ifndef _SINK_STAGE_H
#define _SINK_STAGE_H

#include "sensor_processing_stage.h"

typedef void (*SinkStageCallback)(const struct sensor_data *input, void *user_data);

class SinkStage : public SensorProcessingStage {
public:
    SinkStage(SinkStageCallback callback, void *user_data = nullptr);
    ~SinkStage() override = default;

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    SinkStageCallback callback;
    void *user_data;
};

#endif // _SINK_STAGE_H
