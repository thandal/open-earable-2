#ifndef _SENSOR_PROCESSING_CONSUMER_H
#define _SENSOR_PROCESSING_CONSUMER_H

#include "processing_pipeline.h"

void set_processing_pipeline(const char *name, std::unique_ptr<ProcessingPipeline> pipeline);
ProcessingPipeline* get_processing_pipeline(const char *name);
void remove_processing_pipeline(const char *name);

int sensor_processing_consumer_init(void);

#endif
