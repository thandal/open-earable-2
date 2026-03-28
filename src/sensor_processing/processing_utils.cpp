#include "processing_utils.h"
#include <string.h> // memcpy

float decode_as_float(enum ParseType t, const uint8_t* p) {
    switch (t) {
        case PARSE_TYPE_UINT8:  { uint8_t  v; memcpy(&v, p, sizeof(v)); return (float)v; }
        case PARSE_TYPE_INT8:   { int8_t   v; memcpy(&v, p, sizeof(v)); return (float)v; }
        case PARSE_TYPE_UINT16: { uint16_t v; memcpy(&v, p, sizeof(v)); return (float)v; }
        case PARSE_TYPE_INT16:  { int16_t  v; memcpy(&v, p, sizeof(v)); return (float)v; }
        case PARSE_TYPE_UINT32: { uint32_t v; memcpy(&v, p, sizeof(v)); return (float)v; }
        case PARSE_TYPE_INT32:  { int32_t  v; memcpy(&v, p, sizeof(v)); return (float)v; }
        case PARSE_TYPE_FLOAT:  { float    v; memcpy(&v, p, sizeof(v)); return v; }
        case PARSE_TYPE_DOUBLE: { double   v; memcpy(&v, p, sizeof(v)); return (float)v; }
        default: return 0.0f;
    }
}

template<typename T>
T decode_sensor_data(const struct sensor_data& data) {
    T value;
    memcpy(&value, data.data, sizeof(T));
    return value;
}

template uint8_t decode_sensor_data(const struct sensor_data& data);
template int8_t decode_sensor_data(const struct sensor_data& data);
template uint16_t decode_sensor_data(const struct sensor_data& data);
template int16_t decode_sensor_data(const struct sensor_data& data);
template uint32_t decode_sensor_data(const struct sensor_data& data);
template int32_t decode_sensor_data(const struct sensor_data& data);
template float decode_sensor_data(const struct sensor_data& data);
template double decode_sensor_data(const struct sensor_data& data);

bool check_timestamp_sync(const struct sensor_data *const input[], size_t in_ports, uint64_t threshold_us) {
    uint64_t ref_ts = input[0]->time;

    for (size_t i = 1; i < in_ports; i++) {
        uint64_t ts = input[i]->time;
        uint64_t diff = (ts > ref_ts) ? (ts - ref_ts) : (ref_ts - ts);
        if (diff > threshold_us) {
            return false;
        }
    }
    return true;
}