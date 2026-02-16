/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef _BUTTON_GESTURE_RECOGNIZER_H_
#define _BUTTON_GESTURE_RECOGNIZER_H_

#include "zbus_common.h"

#ifdef __cplusplus
extern "C" {
#endif

int button_gesture_recognizer_init(void);
int button_gesture_recognizer_handle(const struct button_msg *msg);

#ifdef __cplusplus
}
#endif

#endif /* _BUTTON_GESTURE_RECOGNIZER_H_ */
