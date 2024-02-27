#include "app_bluetooth.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>
#include "its.h"

#include <zephyr/logging/log.h>

#define LOG_MODULE_NAME app_bluetooth
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN	(sizeof(DEVICE_NAME) - 1)

static const struct bt_le_conn_param *conn_param = BT_LE_CONN_PARAM(9, 9, 0, 400);

static struct bt_conn *current_conn;

static app_bt_take_picture_cb app_callback_take_picture;
static app_bt_change_resolution_cb app_callback_change_resolution;
static struct its_ble_params_info_t ble_params_info = {.con_interval = 0, .mtu = 23, .tx_phy = 1, .rx_phy = 1};

// In order to maximize data throughput, scale the notifications after the TX data length
static int le_tx_data_length = 20;

enum app_bt_internal_commands {APP_BT_INT_ITS_RX_EVT, APP_BT_INT_SCHEDULE_BLE_PARAMS_INFO_UPDATE};
static struct its_rx_cb_evt_t internal_command_evt;

void schedule_ble_params_info_update(void);

struct app_bt_command {
	enum app_bt_internal_commands command;
	struct its_rx_cb_evt_t its_rx_event;
};
K_MSGQ_DEFINE(msgq_its_rx_commands, sizeof(struct app_bt_command), 8, 4);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_ITS_VAL),
};

void att_mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	LOG_INF("MTU Updated: tx %d, rx %d", tx, rx);
}

static struct bt_gatt_cb gatt_cb = {
	.att_mtu_updated = att_mtu_updated,
};

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	if (!err) {
		int att_mtu = bt_gatt_get_mtu(conn);
		LOG_INF("MTU exchange successful. %i bytes", att_mtu - 3); 
	} else {
		LOG_WRN("MTU exchange failed (err %" PRIu8 ")", err);
	}
}

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

	static struct bt_gatt_exchange_params exchange_params;
	exchange_params.func = mtu_exchange_cb;

	err = bt_gatt_exchange_mtu(conn, &exchange_params);
	if (err) {
		printk("MTU exchange failed (err %d)\n", err);
	} else {
		printk("MTU exchange pending\n");
	}

	err = bt_conn_le_data_len_update(conn, BT_LE_DATA_LEN_PARAM_MAX);
	if (err) {
		LOG_ERR("LE data length update request failed: %d",  err);
	}

	err = bt_conn_le_param_update(conn, conn_param);
	if (err) {
		LOG_ERR("Cannot update connection parameter (err: %d)", err);
	}
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

static void connection_param_update(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	LOG_INF("Con params updated. Connection interval %d ms, latency %i, timeout %i ms", (int) ((float)interval*1.25f), latency, timeout * 10);
	ble_params_info.con_interval = interval;
	schedule_ble_params_info_update();
}

static void le_data_length_updated(struct bt_conn *conn,
				   struct bt_conn_le_data_len_info *info)
{
	LOG_INF("LE data length updated: TX (len: %d time: %d) RX (len: %d time: %d)", 
			info->tx_max_len, info->tx_max_time, info->rx_max_len, info->rx_max_time);

	ble_params_info.mtu = info->tx_max_len;
	schedule_ble_params_info_update();

	// Set the TX data length, which will determine the size of image data transfers. 
	// Subtract 3 bytes for ATT header and 4 bytes for L2CAP header. 
	le_tx_data_length = info->tx_max_len - 7;
	LOG_INF("Notification data length set to %i bytes", le_tx_data_length);
}

static void le_phy_updated(struct bt_conn *conn,
			   struct bt_conn_le_phy_info *param)
{
	LOG_INF("LE PHY updated: TX PHY %i, RX PHY %i\n", param->tx_phy, param->rx_phy);
	ble_params_info.tx_phy = param->tx_phy;
	ble_params_info.rx_phy = param->rx_phy;
	schedule_ble_params_info_update();
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
	.le_param_updated = connection_param_update,
	.le_data_len_updated = le_data_length_updated,
	.le_phy_updated = le_phy_updated,
};

void schedule_ble_params_info_update(void)
{
	internal_command_evt.command = APP_BT_INT_SCHEDULE_BLE_PARAMS_INFO_UPDATE;
	if (k_msgq_put(&msgq_its_rx_commands, &internal_command_evt, K_NO_WAIT) != 0){
		LOG_ERR("RX cmd message queue full!");
	}
}

void its_rx_callback(struct its_rx_cb_evt_t *evt)
{
	static struct app_bt_command bt_cmd = {.command = APP_BT_INT_ITS_RX_EVT};
	bt_cmd.its_rx_event = *evt;
	if (k_msgq_put(&msgq_its_rx_commands, &bt_cmd, K_NO_WAIT) != 0) {
		LOG_ERR("APP BT RX CMD queue full");
	}
}

static struct bt_its_cb its_cb = {
	.rx_cb = its_rx_callback,
};

static void app_bt_thread_func(void)
{
	int err;
	struct app_bt_command app_cmd;
	while (1) {
		k_msgq_get(&msgq_its_rx_commands, &app_cmd, K_FOREVER);

		if(app_cmd.command == APP_BT_INT_ITS_RX_EVT) {
			// Commands originating from the Image Transfer Service
			switch (app_cmd.its_rx_event.command) {
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
					if (app_callback_change_resolution) {
						uint8_t res = app_cmd.its_rx_event.data[0];
						if (res >= 5) res = 6; 
						app_callback_change_resolution(res);
					}
					break;
				case ITS_RX_CMD_CHANGE_PHY:
					LOG_DBG("ITS RX CMD: Change phy");
					err = bt_conn_le_phy_update(current_conn, (app_cmd.its_rx_event.data[0] == 2) ? BT_CONN_LE_PHY_PARAM_2M : BT_CONN_LE_PHY_PARAM_1M);
					if (err) {
						LOG_ERR("Phy update request failed: %d",  err);
					}
					break;
				case ITS_RX_CMD_SEND_BLE_PARAMS:
					LOG_DBG("ITS RX CMD: Send ble params");
					break;
			
				default:
					LOG_ERR("CMD:Invalid command!");
					break;
			}
		} else if(app_cmd.command == APP_BT_INT_SCHEDULE_BLE_PARAMS_INFO_UPDATE) {
			err = bt_its_send_ble_params_info(&ble_params_info);	
			if (err) {
				LOG_ERR("Error sending ble params");
			}
		}
	}
}

int app_bt_init(const struct app_bt_cb *callbacks)
{
	int err = 0;

	if (callbacks) {
		app_callback_take_picture = callbacks->take_picture;
		app_callback_change_resolution = callbacks->change_resolution;
	}

	err = bt_enable(NULL);
	if (err) {
		return err;
	}

	LOG_DBG("Bluetooth initialized");

	bt_gatt_cb_register(&gatt_cb);

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

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
	return bt_its_send_img_data(current_conn, buf, len, le_tx_data_length);
}

K_THREAD_DEFINE(app_bt_thread, 2048, app_bt_thread_func, NULL, NULL, NULL, 
				K_PRIO_PREEMPT(K_LOWEST_APPLICATION_THREAD_PRIO), 0, 0);