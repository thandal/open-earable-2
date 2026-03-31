#include "in_ear_detection_service.h"

#include "macros_common.h"
#include "openearable_common.h"

#include <errno.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(in_ear_detection_service, CONFIG_BLE_LOG_LEVEL);

static struct k_thread thread_data;
static k_tid_t thread_id;

static uint8_t detection_enabled_value = 1U;
static uint8_t wear_state_value = IN_EAR_STATE_UNKNOWN;
static uint8_t is_worn_value = 0U;

static bool detection_enabled_notify_enabled;
static bool wear_state_notify_enabled;
static bool is_worn_notify_enabled;

ZBUS_SUBSCRIBER_DEFINE(in_ear_detection_gatt_sub, CONFIG_BUTTON_MSG_SUB_QUEUE_SIZE);

static K_THREAD_STACK_DEFINE(thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE);

static void detection_enabled_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	detection_enabled_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void wear_state_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	wear_state_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static void is_worn_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	is_worn_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_detection_enabled(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				      void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &detection_enabled_value,
				 sizeof(detection_enabled_value));
}

static ssize_t write_detection_enabled(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				       const void *buf, uint16_t len, uint16_t offset,
				       uint8_t flags)
{
	const uint8_t *value = buf;
	int ret;

	if (len != sizeof(uint8_t)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (offset != 0U) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if ((*value != 0U) && (*value != 1U)) {
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	ret = in_ear_detection_set_enabled(*value == 1U, IN_EAR_UPDATE_SOURCE_MANUAL, 0U);
	if (ret != 0) {
		LOG_WRN("Failed to set in-ear detection enable state: %d", ret);
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}

	return len;
}

static ssize_t read_wear_state(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &wear_state_value,
				 sizeof(wear_state_value));
}

static ssize_t read_is_worn(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &is_worn_value,
				 sizeof(is_worn_value));
}

BT_GATT_SERVICE_DEFINE(in_ear_detection_service,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_IN_EAR_DETECTION_SERVICE),
	BT_GATT_CHARACTERISTIC(BT_UUID_IN_EAR_DETECTION_ENABLE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_detection_enabled, write_detection_enabled,
			       &detection_enabled_value),
	BT_GATT_CCC(detection_enabled_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_IN_EAR_DETECTION_STATE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_wear_state, NULL, &wear_state_value),
	BT_GATT_CCC(wear_state_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_IN_EAR_DETECTION_WORN,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_is_worn, NULL, &is_worn_value),
	BT_GATT_CCC(is_worn_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static void update_characteristics(const struct in_ear_status_msg *status)
{
	detection_enabled_value = status->enabled ? 1U : 0U;
	wear_state_value = (uint8_t)status->state;
	is_worn_value = (status->state == IN_EAR_STATE_WORN) ? 1U : 0U;
}

static void notify_characteristics(void)
{
	int ret;

	if (detection_enabled_notify_enabled) {
		ret = bt_gatt_notify(NULL, &in_ear_detection_service.attrs[2], &detection_enabled_value,
				     sizeof(detection_enabled_value));
		if (ret != 0) {
			LOG_WRN("Failed to notify in-ear detection enabled state: %d", ret);
		}
	}

	if (wear_state_notify_enabled) {
		ret = bt_gatt_notify(NULL, &in_ear_detection_service.attrs[5], &wear_state_value,
				     sizeof(wear_state_value));
		if (ret != 0) {
			LOG_WRN("Failed to notify in-ear detection wear state: %d", ret);
		}
	}

	if (is_worn_notify_enabled) {
		ret = bt_gatt_notify(NULL, &in_ear_detection_service.attrs[8], &is_worn_value,
				     sizeof(is_worn_value));
		if (ret != 0) {
			LOG_WRN("Failed to notify in-ear detection worn flag: %d", ret);
		}
	}
}

static void write_in_ear_detection_gatt(void)
{
	int ret;
	const struct zbus_channel *chan;
	struct in_ear_status_msg status;

	while (1) {
		ret = zbus_sub_wait(&in_ear_detection_gatt_sub, &chan, K_FOREVER);
		ERR_CHK(ret);

		ret = zbus_chan_read(chan, &status, ZBUS_READ_TIMEOUT_MS);
		ERR_CHK(ret);

		update_characteristics(&status);
		notify_characteristics();
	}
}

int init_in_ear_detection_service(void)
{
	int ret;
	struct in_ear_status_msg status;

	ret = in_ear_detection_get(&status);
	if (ret == 0) {
		update_characteristics(&status);
	}

	thread_id = k_thread_create(&thread_data, thread_stack, CONFIG_BUTTON_MSG_SUB_STACK_SIZE,
				    (k_thread_entry_t)write_in_ear_detection_gatt,
				    NULL, NULL, NULL,
				    K_PRIO_PREEMPT(CONFIG_BUTTON_MSG_SUB_THREAD_PRIO), 0, K_NO_WAIT);

	ret = k_thread_name_set(thread_id, "IN_EAR_GATT_SUB");
	if (ret != 0) {
		LOG_ERR("Failed to create in-ear detection GATT thread");
		return ret;
	}

	ret = in_ear_detection_subscribe(&in_ear_detection_gatt_sub, ZBUS_ADD_OBS_TIMEOUT_MS);
	if (ret != 0) {
		LOG_ERR("Failed to subscribe in-ear detection GATT service");
		return ret;
	}

	return 0;
}
