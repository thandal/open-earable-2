#ifndef _TRIGGER_ACTION_STAGE_H
#define _TRIGGER_ACTION_STAGE_H

#include "ParseType.h"
#include "sensor_processing_stage.h"

typedef void (*TriggerActionCallback)(const struct sensor_data *trigger, void *user_data);

class TriggerActionStage : public SensorProcessingStage {
public:
    TriggerActionStage(enum ParseType trigger_type,
                       TriggerActionCallback callback,
                       void *user_data = nullptr);
    ~TriggerActionStage() override = default;

    int process(const struct sensor_data *const input[], struct sensor_data *output) override;

private:
    enum ParseType trigger_type;
    TriggerActionCallback callback;
    void *user_data;
};

#endif // _TRIGGER_ACTION_STAGE_H
