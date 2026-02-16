/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "media_control.h"

#include "bt_content_ctrl.h"

#include <zephyr/kernel.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(media_control, CONFIG_MAIN_LOG_LEVEL);

struct media_control_mapping {
	enum media_control_gesture gesture;
	enum media_control_action action;
};

ZBUS_SUBSCRIBER_DEFINE(media_control_evt_sub, CONFIG_MEDIA_CONTROL_MSG_SUB_QUEUE_SIZE);

ZBUS_CHAN_DEFINE(media_control_chan, struct media_control_msg, NULL, NULL, ZBUS_OBSERVERS_EMPTY,
		 ZBUS_MSG_INIT(0));

static struct k_thread media_control_msg_sub_thread_data;
static k_tid_t media_control_msg_sub_thread_id;

K_THREAD_STACK_DEFINE(media_control_msg_sub_thread_stack, CONFIG_MEDIA_CONTROL_MSG_SUB_STACK_SIZE);

/* Central mapping table: update this to remap button gestures. */
static const struct media_control_mapping gesture_action_map[] = {
	{ MEDIA_CTRL_GESTURE_SINGLE_PRESS, MEDIA_CTRL_ACTION_PLAY_PAUSE_TOGGLE },
	{ MEDIA_CTRL_GESTURE_DOUBLE_PRESS, MEDIA_CTRL_ACTION_NEXT_TRACK },
	{ MEDIA_CTRL_GESTURE_TRIPLE_PRESS, MEDIA_CTRL_ACTION_PREV_TRACK },
};

static int media_control_execute(enum media_control_action action)
{
	switch (action) {
	case MEDIA_CTRL_ACTION_PLAY_PAUSE_TOGGLE:
		if (IS_ENABLED(CONFIG_WALKIE_TALKIE_DEMO)) {
			LOG_WRN("Play/pause is not supported in walkie-talkie mode");
			return -ENOTSUP;
		}

		if (bt_content_ctlr_media_state_playing()) {
			return bt_content_ctrl_stop(NULL);
		}

		return bt_content_ctrl_start(NULL);

	case MEDIA_CTRL_ACTION_NEXT_TRACK:
		return bt_content_ctrl_next(NULL);

	case MEDIA_CTRL_ACTION_PREV_TRACK:
		return bt_content_ctrl_prev(NULL);

	default:
		return -EINVAL;
	}
}

static void media_control_msg_sub_thread(void)
{
	int ret;
	const struct zbus_channel *chan;
	struct media_control_msg msg;

	while (1) {
		ret = zbus_sub_wait(&media_control_evt_sub, &chan, K_FOREVER);
		if (ret) {
			LOG_WRN("zbus_sub_wait failed: %d", ret);
			continue;
		}

		ret = zbus_chan_read(chan, &msg, K_MSEC(100));
		if (ret) {
			LOG_WRN("zbus_chan_read failed: %d", ret);
			continue;
		}

		ret = media_control_execute(msg.action);
		if (ret) {
			LOG_WRN("Failed to execute action %d: %d", msg.action, ret);
		}
	}
}

int media_control_init(void)
{
	int ret;

	media_control_msg_sub_thread_id =
		k_thread_create(&media_control_msg_sub_thread_data, media_control_msg_sub_thread_stack,
				CONFIG_MEDIA_CONTROL_MSG_SUB_STACK_SIZE,
				(k_thread_entry_t)media_control_msg_sub_thread, NULL, NULL, NULL,
				K_PRIO_PREEMPT(CONFIG_MEDIA_CONTROL_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);
	ret = k_thread_name_set(media_control_msg_sub_thread_id, "MEDIA_CTRL_SUB");
	if (ret) {
		return ret;
	}

	ret = zbus_chan_add_obs(&media_control_chan, &media_control_evt_sub, K_MSEC(200));
	if (ret) {
		return ret;
	}

	return 0;
}

int media_control_submit_action(enum media_control_action action)
{
	struct media_control_msg msg = {
		.action = action,
	};

	return zbus_chan_pub(&media_control_chan, &msg, K_NO_WAIT);
}

int media_control_handle_gesture(enum media_control_gesture gesture)
{
	for (size_t i = 0; i < ARRAY_SIZE(gesture_action_map); i++) {
		if (gesture_action_map[i].gesture == gesture) {
			return media_control_submit_action(gesture_action_map[i].action);
		}
	}

	LOG_WRN("Unhandled gesture: %d", gesture);
	return -EINVAL;
}
