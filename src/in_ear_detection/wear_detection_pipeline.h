#ifndef _WEAR_DETECTION_PIPELINE_H_
#define _WEAR_DETECTION_PIPELINE_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

int in_ear_detection_pipeline_init(void);
void in_ear_detection_pipeline_on_enabled_changed(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* _WEAR_DETECTION_PIPELINE_H_ */
