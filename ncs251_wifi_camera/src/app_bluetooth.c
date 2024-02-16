#include "app_bluetooth.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include "its.h"

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME app_bluetooth
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

static struct bt_conn *current_conn;
static app_bt_take_picture_cb app_callback_take_picture;

K_MSGQ_DEFINE(msgq_its_rx_commands, sizeof(struct its_rx_cb_evt_t), 8, 4);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ITS_VAL),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err) {
		LOG_ERR("Connection failed (err %u)", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("Connected %s", addr);

	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_INF("Disconnected: %s (reason %u)", addr, reason);

	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

void its_rx_callback(struct its_rx_cb_evt_t *evt)
{
	if (k_msgq_put(&msgq_its_rx_commands, evt, K_NO_WAIT) != 0) {
		LOG_ERR("APP BT RX CMD queue full");
	}
}

static struct bt_its_cb its_cb = {
	.rx_cb = its_rx_callback,
};

static void app_bt_thread_func(void)
{
	struct its_rx_cb_evt_t rx_evt;
	while (1) {
		k_msgq_get(&msgq_its_rx_commands, &rx_evt, K_FOREVER);

		switch (rx_evt.command) {
			case ITS_RX_CMD_SINGLE_CAPTURE:
				LOG_DBG("ITS RX CMD: SingleCapture");
				if (app_callback_take_picture) {
					app_callback_take_picture();
				}
				break;
			case ITS_RX_CMD_START_STREAM:
				LOG_DBG("ITS RX CMD: Start Stream");
				break;
			case ITS_RX_CMD_STOP_STREAM:
				LOG_DBG("ITS RX CMD: Stop stream");
				break;
			case ITS_RX_CMD_CHANGE_RESOLUTION:
				LOG_DBG("ITS RX CMD: Change res");
				break;
			case ITS_RX_CMD_CHANGE_PHY:
				LOG_DBG("ITS RX CMD: Change phy");
				break;
			case ITS_RX_CMD_SEND_BLE_PARAMS:
				LOG_DBG("ITS RX CMD: Send ble params");
				break;
			default:
				LOG_ERR("CMD:Invalid command!");
				break;
		}
		if(rx_evt.len > 0) LOG_HEXDUMP_INF(rx_evt.data, rx_evt.len, "Data: ");
	}
}

int app_bt_init(const struct app_bt_cb *callbacks)
{
	int err = 0;

	if (callbacks) {
		app_callback_take_picture = callbacks->take_picture;
	}

	err = bt_enable(NULL);
	if (err) {
		return err;
	}

	LOG_INF("Bluetooth initialized");

	//if (IS_ENABLED(CONFIG_SETTINGS)) {
	//	settings_load();
	//}

    err = bt_its_init(&its_cb);
	if (err) {
		LOG_ERR("Failed to initialize Image Transfer Service (err: %d)", err);
		return 0;
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("Advertising failed to start (err %d)", err);
		return err;
	}

    return 0;
}

int app_bt_send_picture_header(uint32_t pic_size)
{
	struct its_img_info_t img_info = {.file_size_bytes = pic_size};
	return bt_its_send_img_info(&img_info);
}

int app_bt_send_picture_data(uint8_t *buf, uint16_t len)
{
	return bt_its_send_img_data(buf, len);
}

K_THREAD_DEFINE(app_bt_thread, 1024, app_bt_thread_func, NULL, NULL, NULL, 7, 0, 0);