/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *  @brief LED Button Service (ITS) sample
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include "its.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_its, LOG_LEVEL_INF);

static bool                   notify_enabled[2];
static struct bt_its_cb       its_cb;
static struct its_rx_cb_evt_t cb_evt;
static struct its_ble_params_info_t scheduled_params = {.con_interval = 0};

static void its_tx_ccc_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	LOG_DBG("Notify updated on TX char, handle %i, value %i", attr->handle, value);
	notify_enabled[0] = (value == BT_GATT_CCC_NOTIFY);
}

static void its_img_info_ccc_changed(const struct bt_gatt_attr *attr,
				  uint16_t value)
{
	LOG_DBG("Notify updated on img info char, handle %i, value %i", attr->handle, value);
	notify_enabled[1] = (value == BT_GATT_CCC_NOTIFY);
	if (scheduled_params.con_interval != 0) {
		bt_its_send_ble_params_info(&scheduled_params);
		scheduled_params.con_interval = 0;
	}
}

static ssize_t its_rx_received(struct bt_conn *conn,
			 const struct bt_gatt_attr *attr,
			 const void *buf,
			 uint16_t len, uint16_t offset, uint8_t flags)
{
	LOG_DBG("Attribute write, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	if (offset != 0) {
		LOG_DBG("Write RX char: Incorrect data offset");
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	uint8_t *byte_ptr = (uint8_t*)buf;

	if (its_cb.rx_cb) {
		cb_evt.command = byte_ptr[0];
		cb_evt.data = &byte_ptr[1];
		cb_evt.len = len - 1;
		its_cb.rx_cb(&cb_evt);
	}

	return len;
}

#ifdef CONFIG_BT_ITS_POLL_BUTTON
static ssize_t read_button(struct bt_conn *conn,
			  const struct bt_gatt_attr *attr,
			  void *buf,
			  uint16_t len,
			  uint16_t offset)
{
	const char *value = attr->user_data;

	LOG_DBG("Attribute read, handle: %u, conn: %p", attr->handle,
		(void *)conn);

	if (its_cb.button_cb) {
		button_state = its_cb.button_cb();
		return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
					 sizeof(*value));
	}

	return 0;
}
#endif

/* LED Button Service Declaration */
BT_GATT_SERVICE_DEFINE(its_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_ITS), 					    // Attr index: 0
	BT_GATT_CHARACTERISTIC(BT_UUID_ITS_TX,					// Attr index: 1,2
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(its_tx_ccc_changed,							// Attr index: 3
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_ITS_RX,					// Attr index: 4,5
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE,
			       NULL, its_rx_received, NULL),
	BT_GATT_CHARACTERISTIC(BT_UUID_ITS_IMG_INFO,			// Attr index: 6,7
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(its_img_info_ccc_changed,				    // Attr index: 8
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

int bt_its_init(struct bt_its_cb *callbacks)
{
	if (callbacks) {
		its_cb.rx_cb = callbacks->rx_cb;
	}

	return 0;
}

int bt_its_send_img_data(struct bt_conn *conn, uint8_t *buf, uint16_t length, uint16_t le_max_length)
{
	int err;
	uint8_t *current_buf = buf;

	LOG_DBG("IMG buf length %i", length);
	while (length > le_max_length) {
		LOG_DBG("Notify TX: Len %i, Remaining: %i", le_max_length, length);
		err = bt_gatt_notify(NULL, &its_svc.attrs[2], current_buf, le_max_length);
		if (err < 0) {
			LOG_ERR("BT notify error: %i", err);
		}
		current_buf += le_max_length;
		length -= le_max_length;
	}

	LOG_DBG("Notify TX remaining %i bytes", length);
	err = bt_gatt_notify(NULL, &its_svc.attrs[2], current_buf, length);
	if (err < 0) {
		LOG_ERR("BT notify error: %i", err);
	}

	return 0;
}

int bt_its_send_img_info(struct its_img_info_t * img_info)
{
	uint8_t notify_buf[1 + sizeof(struct its_img_info_t)];

	if (!notify_enabled[1]) {
		return -EACCES;
	}

	notify_buf[0] = ITS_IMG_INFO_DATA_TYPE_IMG_INFO;
	memcpy(&notify_buf[1], img_info, sizeof(struct its_img_info_t));
	
	LOG_DBG("IMG info Notify TX %i bytes. Img size: %i", 1 + sizeof(struct its_img_info_t), img_info->file_size_bytes);

	return bt_gatt_notify(NULL, &its_svc.attrs[7], 
						  notify_buf, 1 + sizeof(struct its_img_info_t));
}

int bt_its_send_ble_params_info(struct its_ble_params_info_t* ble_params_info)
{
	uint8_t notify_buf[1 + sizeof(struct its_ble_params_info_t)];

	if (!notify_enabled[1]) {
		// If notifications have not yet been enabled, schedule the params to be sent later
		scheduled_params = *ble_params_info;
		LOG_DBG("Scheduling params info for later");
		
		return 0;
	}

	notify_buf[0] = ITS_IMG_INFO_DATA_TYPE_BLE_PARAMS;
	memcpy(&notify_buf[1], ble_params_info, sizeof(struct its_ble_params_info_t));
	
	LOG_DBG("BLE params Notify TX %i bytes",  1 + sizeof(struct its_ble_params_info_t));
	
	return bt_gatt_notify(NULL, &its_svc.attrs[7], 
						  notify_buf, 1 + sizeof(struct its_ble_params_info_t));
}

