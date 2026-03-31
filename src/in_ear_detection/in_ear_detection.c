#include "in_ear_detection.h"

#include <errno.h>

#include "openearable_common.h"
#include "wear_detection_pipeline.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(in_ear_detection, LOG_LEVEL_INF);

/*
 * Serialize read-modify-publish updates so change counters and transition
 * metadata remain consistent even if multiple producers write concurrently.
 */
K_MUTEX_DEFINE(in_ear_detection_lock);

/*
 * Channel-local storage holds the global in-ear state. The default payload keeps
 * the framework in a defined "unknown" state until the first real decision is
 * published by a producer.
 */
ZBUS_CHAN_DEFINE(in_ear_detection_chan, struct in_ear_status_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(
			 .enabled = true,
			 .previous_enabled = true,
			 .state = IN_EAR_STATE_UNKNOWN,
			 .previous_state = IN_EAR_STATE_UNKNOWN,
			 .change_counter = 0,
			 .timestamp_us = 0,
			 .source = IN_EAR_UPDATE_SOURCE_UNSPECIFIED,
			 .flags = 0));

static int in_ear_detection_read_current(struct in_ear_status_msg *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	return zbus_chan_read(&in_ear_detection_chan, status, K_NO_WAIT);
}

int in_ear_detection_init(void)
{
	return 0;
}

bool in_ear_detection_is_enabled(void)
{
	struct in_ear_status_msg status;
	int ret = in_ear_detection_read_current(&status);

	if (ret != 0) {
		LOG_WRN("Failed to read in-ear enable state: %d", ret);
		return false;
	}

	return status.enabled;
}

enum in_ear_state in_ear_detection_state(void)
{
	struct in_ear_status_msg status;
	int ret = in_ear_detection_read_current(&status);

	if (ret != 0) {
		LOG_WRN("Failed to read in-ear state: %d", ret);
		return IN_EAR_STATE_UNKNOWN;
	}

	return status.state;
}

bool in_ear_detection_is_worn(void)
{
	return in_ear_detection_state() == IN_EAR_STATE_WORN;
}

int in_ear_detection_get(struct in_ear_status_msg *status)
{
	return in_ear_detection_read_current(status);
}

int in_ear_detection_publish(const struct in_ear_status_msg *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	return zbus_chan_pub(&in_ear_detection_chan, status, K_FOREVER);
}

int in_ear_detection_set_enabled(bool enabled, uint16_t source, uint16_t flags)
{
	LOG_DBG("Setting in-ear detection %s (source=%u, flags=0x%04x)", enabled ? "enabled" : "disabled", source, flags);

	struct in_ear_status_msg current;
	struct in_ear_status_msg next;
	int ret;
	bool changed = false;

	ret = k_mutex_lock(&in_ear_detection_lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	ret = in_ear_detection_read_current(&current);
	if (ret != 0) {
		goto done;
	}

	if (current.enabled == enabled) {
		ret = 0;
		goto done;
	}

	next = current;
	next.previous_enabled = current.enabled;
	next.enabled = enabled;
	next.source = source;
	next.flags = flags;
	next.timestamp_us = oe_micros();
	next.change_counter = current.change_counter + 1U;
	changed = true;

	ret = in_ear_detection_publish(&next);

done:
	k_mutex_unlock(&in_ear_detection_lock);
	if (ret == 0 && changed) {
		in_ear_detection_pipeline_on_enabled_changed(enabled);
	}
	return ret;
}

int in_ear_detection_enable(uint16_t source, uint16_t flags)
{
	return in_ear_detection_set_enabled(true, source, flags);
}

int in_ear_detection_disable(uint16_t source, uint16_t flags)
{
	return in_ear_detection_set_enabled(false, source, flags);
}

int in_ear_detection_set_state(enum in_ear_state state, uint16_t source, uint16_t flags)
{
	struct in_ear_status_msg current;
	struct in_ear_status_msg next;
	int ret;

	ret = k_mutex_lock(&in_ear_detection_lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	ret = in_ear_detection_read_current(&current);
	if (ret != 0) {
		goto done;
	}

	if (!current.enabled) {
		ret = 0;
		goto done;
	}

	if (current.state == state) {
		ret = 0;
		goto done;
	}

	next = current;
	next.previous_state = current.state;
	next.state = state;
	next.source = source;
	next.flags = flags;
	next.timestamp_us = oe_micros();
	next.change_counter = current.change_counter + 1U;

	ret = in_ear_detection_publish(&next);

done:
	k_mutex_unlock(&in_ear_detection_lock);
	return ret;
}

int in_ear_detection_subscribe(const struct zbus_observer *observer, k_timeout_t timeout)
{
	if (observer == NULL) {
		return -EINVAL;
	}

	return zbus_chan_add_obs(&in_ear_detection_chan, observer, timeout);
}

const struct zbus_channel *in_ear_detection_channel(void)
{
	return &in_ear_detection_chan;
}
