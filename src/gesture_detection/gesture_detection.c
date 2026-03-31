#include "gesture_detection.h"

#include <errno.h>

#include "gesture_detection_pipeline.h"
#include "openearable_common.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gesture_detection, LOG_LEVEL_DBG);

K_MUTEX_DEFINE(gesture_detection_lock);

ZBUS_CHAN_DEFINE(gesture_detection_chan, struct gesture_detection_event_msg, NULL, NULL,
		 ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(
			 .enabled = true,
			 .previous_enabled = true,
			 .tap_count = 0,
			 .previous_tap_count = 0,
			 .change_counter = 0,
			 .timestamp_us = 0,
			 .source = GESTURE_UPDATE_SOURCE_UNSPECIFIED,
			 .flags = 0));

static int gesture_detection_read_current(struct gesture_detection_event_msg *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	return zbus_chan_read(&gesture_detection_chan, status, K_NO_WAIT);
}

int gesture_detection_init(void)
{
	LOG_INF("Gesture detection framework initialized");
	return 0;
}

bool gesture_detection_is_enabled(void)
{
	struct gesture_detection_event_msg status;
	int ret = gesture_detection_read_current(&status);

	if (ret != 0) {
		LOG_WRN("Failed to read gesture detection enable state: %d", ret);
		return false;
	}

	return status.enabled;
}

int gesture_detection_get(struct gesture_detection_event_msg *status)
{
	return gesture_detection_read_current(status);
}

int gesture_detection_publish(const struct gesture_detection_event_msg *status)
{
	if (status == NULL) {
		return -EINVAL;
	}

	return zbus_chan_pub(&gesture_detection_chan, status, K_FOREVER);
}

int gesture_detection_set_enabled(bool enabled, uint16_t source, uint16_t flags)
{
	struct gesture_detection_event_msg current;
	struct gesture_detection_event_msg next;
	int ret;
	bool changed = false;

	ret = k_mutex_lock(&gesture_detection_lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	ret = gesture_detection_read_current(&current);
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

	LOG_INF("Gesture detection %s (source=%u flags=0x%04x)",
		enabled ? "enabled" : "disabled", source, flags);

	ret = gesture_detection_publish(&next);

done:
	k_mutex_unlock(&gesture_detection_lock);
	if (ret == 0 && changed) {
		gesture_detection_pipeline_on_enabled_changed(enabled);
	}
	return ret;
}

int gesture_detection_enable(uint16_t source, uint16_t flags)
{
	return gesture_detection_set_enabled(true, source, flags);
}

int gesture_detection_disable(uint16_t source, uint16_t flags)
{
	return gesture_detection_set_enabled(false, source, flags);
}

int gesture_detection_publish_event(uint8_t tap_count, uint16_t source, uint16_t flags)
{
	struct gesture_detection_event_msg current;
	struct gesture_detection_event_msg next;
	int ret;

	if (tap_count == 0U) {
		return -EINVAL;
	}

	ret = k_mutex_lock(&gesture_detection_lock, K_FOREVER);
	if (ret != 0) {
		return ret;
	}

	ret = gesture_detection_read_current(&current);
	if (ret != 0) {
		goto done;
	}

	if (!current.enabled) {
		ret = 0;
		goto done;
	}

	next = current;
	next.previous_tap_count = current.tap_count;
	next.tap_count = tap_count;
	next.source = source;
	next.flags = flags;
	next.timestamp_us = oe_micros();
	next.change_counter = current.change_counter + 1U;

	LOG_INF("Publishing gesture event tap_count=%u source=%u flags=0x%04x at %llu us",
		tap_count, source, flags, (unsigned long long)next.timestamp_us);

	ret = gesture_detection_publish(&next);

done:
	k_mutex_unlock(&gesture_detection_lock);
	return ret;
}

int gesture_detection_subscribe(const struct zbus_observer *observer, k_timeout_t timeout)
{
	if (observer == NULL) {
		return -EINVAL;
	}

	return zbus_chan_add_obs(&gesture_detection_chan, observer, timeout);
}

const struct zbus_channel *gesture_detection_channel(void)
{
	return &gesture_detection_chan;
}
