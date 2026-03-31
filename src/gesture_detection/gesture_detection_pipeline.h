#ifndef _GESTURE_DETECTION_PIPELINE_H_
#define _GESTURE_DETECTION_PIPELINE_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int gesture_detection_pipeline_init(void);
void gesture_detection_pipeline_on_enabled_changed(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* _GESTURE_DETECTION_PIPELINE_H_ */
