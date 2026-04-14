#ifndef SENSOR_SINK_H
#define SENSOR_SINK_H

#include "openearable_common.h"

#ifdef __cplusplus
extern "C" {
#endif

int sensor_sink_put(const struct sensor_msg *msg);

#ifdef __cplusplus
}
#endif

#endif
