#ifndef IN_EAR_DETECTION_SERVICE_H
#define IN_EAR_DETECTION_SERVICE_H

#include <zephyr/bluetooth/gatt.h>

#include "in_ear_detection.h"

#define BT_UUID_IN_EAR_DETECTION_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xf19f2a10, 0x8d69, 0x4ec5, 0xb64c, 0x0242ac120002)

#define BT_UUID_IN_EAR_DETECTION_ENABLE_VAL \
	BT_UUID_128_ENCODE(0xf19f2a11, 0x8d69, 0x4ec5, 0xb64c, 0x0242ac120002)

#define BT_UUID_IN_EAR_DETECTION_STATE_VAL \
	BT_UUID_128_ENCODE(0xf19f2a12, 0x8d69, 0x4ec5, 0xb64c, 0x0242ac120002)

#define BT_UUID_IN_EAR_DETECTION_WORN_VAL \
	BT_UUID_128_ENCODE(0xf19f2a13, 0x8d69, 0x4ec5, 0xb64c, 0x0242ac120002)

#define BT_UUID_IN_EAR_DETECTION_SERVICE \
	BT_UUID_DECLARE_128(BT_UUID_IN_EAR_DETECTION_SERVICE_VAL)

#define BT_UUID_IN_EAR_DETECTION_ENABLE \
	BT_UUID_DECLARE_128(BT_UUID_IN_EAR_DETECTION_ENABLE_VAL)

#define BT_UUID_IN_EAR_DETECTION_STATE \
	BT_UUID_DECLARE_128(BT_UUID_IN_EAR_DETECTION_STATE_VAL)

#define BT_UUID_IN_EAR_DETECTION_WORN \
	BT_UUID_DECLARE_128(BT_UUID_IN_EAR_DETECTION_WORN_VAL)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the in-ear detection GATT service.
 *
 * The service exposes:
 * - a read/write/notify characteristic for enabling or disabling detection
 * - a read/notify characteristic for the raw in-ear state enum
 * - a read/notify characteristic for the boolean "is worn" view
 *
 * @retval 0 Service initialized successfully.
 * @retval Negative error code on failure.
 */
int init_in_ear_detection_service(void);

#ifdef __cplusplus
}
#endif

#endif
