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
    LOG_INF("");
    switch (evt->command) {
        case ITS_RX_CMD_SINGLE_CAPTURE:
            break;
        case ITS_RX_CMD_SEND_BLE_PARAMS:
            break;
        case ITS_RX_CMD_START_STREAM:
            break;
        case ITS_RX_CMD_STOP_STREAM:
            break;
        case ITS_RX_CMD_CHANGE_RESOLUTION:
            break;
        case ITS_RX_CMD_CHANGE_PHY:
            break;
    }
}

static struct bt_its_cb its_cb = {
	.rx_cb = its_rx_callback,
};

int app_bt_init(void)
{
	int err = 0;

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