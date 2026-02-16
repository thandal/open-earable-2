/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "button_gesture_recognizer.h"

#include "button_assignments.h"
#include "media_control.h"

#include <stdint.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(button_gesture_recognizer, CONFIG_MAIN_LOG_LEVEL);

#define MULTIPRESS_WINDOW K_MSEC(CONFIG_BUTTON_GESTURE_MULTIPRESS_WINDOW_MS)

static uint8_t press_count;
static struct k_work_delayable gesture_resolve_work;

static enum media_control_gesture gesture_from_count(uint8_t count)
{
	switch (count) {
	case 1:
		return MEDIA_CTRL_GESTURE_SINGLE_PRESS;
	case 2:
		return MEDIA_CTRL_GESTURE_DOUBLE_PRESS;
	case 3:
		return MEDIA_CTRL_GESTURE_TRIPLE_PRESS;
	default:
		return 0;
	}
}

static void gesture_resolve_work_handler(struct k_work *work)
{
	enum media_control_gesture gesture;
	int ret;

	ARG_UNUSED(work);

	gesture = gesture_from_count(press_count);
	press_count = 0;

	if (gesture == 0) {
		return;
	}

	ret = media_control_handle_gesture(gesture);
	if (ret) {
		LOG_WRN("Failed to handle gesture %d: %d", gesture, ret);
	}
}

int button_gesture_recognizer_init(void)
{
	press_count = 0;
	k_work_init_delayable(&gesture_resolve_work, gesture_resolve_work_handler);
	return 0;
}

int button_gesture_recognizer_handle(const struct button_msg *msg)
{
	if (msg == NULL) {
		return -EINVAL;
	}

	if (msg->button_pin != BUTTON_PLAY_PAUSE) {
		return 0;
	}

	if (msg->button_action != BUTTON_RELEASED) {
		return 0;
	}

	if (press_count < UINT8_MAX) {
		press_count++;
	}

	k_work_reschedule(&gesture_resolve_work, MULTIPRESS_WINDOW);
	return 0;
}
