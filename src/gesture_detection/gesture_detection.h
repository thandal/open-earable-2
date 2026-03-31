#ifndef _GESTURE_DETECTION_H_
#define _GESTURE_DETECTION_H_

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>

#ifdef __cplusplus
extern "C" {
#endif

enum gesture_detection_update_source {
	GESTURE_UPDATE_SOURCE_UNSPECIFIED = 0,
	GESTURE_UPDATE_SOURCE_PIPELINE,
	GESTURE_UPDATE_SOURCE_MANUAL,
};

struct gesture_detection_event_msg {
	bool enabled;
	bool previous_enabled;
	uint8_t tap_count;
	uint8_t previous_tap_count;
	uint32_t change_counter;
	uint64_t timestamp_us;
	uint16_t source;
	uint16_t flags;
};

int gesture_detection_init(void);
bool gesture_detection_is_enabled(void);
int gesture_detection_set_enabled(bool enabled, uint16_t source, uint16_t flags);
int gesture_detection_enable(uint16_t source, uint16_t flags);
int gesture_detection_disable(uint16_t source, uint16_t flags);
int gesture_detection_get(struct gesture_detection_event_msg *status);
int gesture_detection_publish(const struct gesture_detection_event_msg *status);
int gesture_detection_publish_event(uint8_t tap_count, uint16_t source, uint16_t flags);
int gesture_detection_subscribe(const struct zbus_observer *observer, k_timeout_t timeout);
const struct zbus_channel *gesture_detection_channel(void);

ZBUS_CHAN_DECLARE(gesture_detection_chan);

#ifdef __cplusplus
}
#endif

#endif /* _GESTURE_DETECTION_H_ */
