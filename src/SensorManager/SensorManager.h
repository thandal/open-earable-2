#ifndef _SENSOR_MANAGER_H
#define _SENSOR_MANAGER_H

#include "openearable_common.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

enum sensor_manager_state {
    INIT,
    RUNNING,
    SUSPENDED,
};

extern struct k_work_q sensor_work_q;

enum sensor_manager_state get_state();

void init_sensor_manager();

void start_sensor_manager();

void stop_sensor_manager();

void config_sensor(struct sensor_config * config);
int get_sensor_configuration(uint8_t sensor_id, struct sensor_config *config_out);
int update_sensor_consumer_state(uint8_t sensor_id, uint8_t consumer_mask, bool enabled,
                                 uint8_t preferred_sample_rate_idx);

#ifdef __cplusplus
}
#endif

#endif
