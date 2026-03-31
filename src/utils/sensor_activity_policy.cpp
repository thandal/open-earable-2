#include "sensor_activity_policy.h"

#include <errno.h>

#include "../../SensorManager/SensorManager.h"
#include "in_ear_detection.h"
#include "SensorScheme.h"

#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_activity_policy, LOG_LEVEL_INF);

static void sensor_config_evt_handler(const struct zbus_channel *chan);
ZBUS_LISTENER_DEFINE(sensor_config_evt_listen, sensor_config_evt_handler);

static void handle_in_ear_detection_sensors()
{
	sensor_config baro_config = {};
	sensor_config ppg_config = {};
	int ret = get_sensor_configuration(ID_TEMP_BARO, &baro_config);
	if (ret != 0) {
		LOG_ERR("Failed to get barometer sensor configuration: %d", ret);
		return;
	}
	ret = get_sensor_configuration(ID_PPG, &ppg_config);
	if (ret != 0) {
		LOG_ERR("Failed to get PPG sensor configuration: %d", ret);
		return;
	}

	LOG_DBG("Barometer config: sampleRateIndex=%u, storageOptions=0x%02x", baro_config.sampleRateIndex, baro_config.storageOptions);
	LOG_DBG("PPG config: sampleRateIndex=%u, storageOptions=0x%02x", ppg_config.sampleRateIndex, ppg_config.storageOptions);
	// check if other storage options than processing are active for barometer or ppg
	const bool ppg_active = (ppg_config.storageOptions & ~SENSOR_CONSUMER_PROCESSING) != 0;
	const bool baro_active = (baro_config.storageOptions & ~SENSOR_CONSUMER_PROCESSING) != 0;
	
	LOG_DBG("PPG active: %s, Barometer active: %s", ppg_active ? "true" : "false", baro_active ? "true" : "false");

	const bool active = ppg_active || baro_active;

	if (active) {
		(void)in_ear_detection_disable(IN_EAR_UPDATE_SOURCE_UNSPECIFIED, 0U);
	} else {
		(void)in_ear_detection_enable(IN_EAR_UPDATE_SOURCE_UNSPECIFIED, 0U);
	}
}

static void sensor_config_evt_handler(const struct zbus_channel *chan)
{
	if (chan == NULL || chan->message == NULL || chan->message_size != sizeof(struct sensor_config)) {
		LOG_ERR("Invalid sensor config event");
		return;
	}
	const struct sensor_config *config = static_cast<const struct sensor_config *>(chan->message);
	LOG_DBG("Received sensor config event: sensorId=%u, sampleRateIndex=%u, storageOptions=0x%02x",
		config->sensorId, config->sampleRateIndex, config->storageOptions);
	if (config->sensorId == ID_PPG || config->sensorId == ID_TEMP_BARO) {
		handle_in_ear_detection_sensors();
	}
}

int sensor_activity_policy_init(void)
{
	int ret = zbus_chan_add_obs(&sensor_config_chan, &sensor_config_evt_listen,
				    ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret != 0 && ret != -EALREADY) {
		LOG_ERR("Failed to subscribe sensor activity policy: %d", ret);
		return ret;
	}

	handle_in_ear_detection_sensors();
	return 0;
}
