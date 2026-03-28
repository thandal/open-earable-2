#ifndef _PROCESSING_UTILS_H
#define _PROCESSING_UTILS_H

#include "openearable_common.h"

#include "ParseType.h"

float decode_as_float(enum ParseType t, const uint8_t *p);

template<typename T>
T decode_sensor_data(const struct sensor_data& data);

bool check_timestamp_sync(const struct sensor_data *const input[], size_t in_ports, uint64_t threshold_us);

#endif // _PROCESSING_UTILS_H
