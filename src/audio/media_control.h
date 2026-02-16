/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _MEDIA_CONTROL_H_
#define _MEDIA_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

enum media_control_gesture {
	MEDIA_CTRL_GESTURE_SINGLE_PRESS = 1,
	MEDIA_CTRL_GESTURE_DOUBLE_PRESS,
	MEDIA_CTRL_GESTURE_TRIPLE_PRESS,
};

enum media_control_action {
	MEDIA_CTRL_ACTION_PLAY_PAUSE_TOGGLE = 1,
	MEDIA_CTRL_ACTION_NEXT_TRACK,
	MEDIA_CTRL_ACTION_PREV_TRACK,
};

struct media_control_msg {
	enum media_control_action action;
};

int media_control_init(void);
int media_control_submit_action(enum media_control_action action);
int media_control_handle_gesture(enum media_control_gesture gesture);

#ifdef __cplusplus
}
#endif

#endif /* _MEDIA_CONTROL_H_ */
