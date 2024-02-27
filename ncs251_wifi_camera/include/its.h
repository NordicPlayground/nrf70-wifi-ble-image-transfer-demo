/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef __BT_ITS_H
#define __BT_ITS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/types.h>

#define BT_UUID_ITS_VAL \
	BT_UUID_128_ENCODE(0x6E400001, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA3E)

#define BT_UUID_ITS_TX_VAL \
	BT_UUID_128_ENCODE(0x6E400003, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA3E)

#define BT_UUID_ITS_RX_VAL \
	BT_UUID_128_ENCODE(0x6E400002, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA3E)

#define BT_UUID_ITS_IMG_INFO_VAL \
	BT_UUID_128_ENCODE(0x6E400004, 0xB5A3, 0xF393, 0xE0A9, 0xE50E24DCCA3E)

#define BT_UUID_ITS           BT_UUID_DECLARE_128(BT_UUID_ITS_VAL)
#define BT_UUID_ITS_TX    	  BT_UUID_DECLARE_128(BT_UUID_ITS_TX_VAL)
#define BT_UUID_ITS_RX        BT_UUID_DECLARE_128(BT_UUID_ITS_RX_VAL)
#define BT_UUID_ITS_IMG_INFO  BT_UUID_DECLARE_128(BT_UUID_ITS_IMG_INFO_VAL)

enum its_rx_cmd_e {ITS_RX_CMD_SINGLE_CAPTURE = 1, 
				   ITS_RX_CMD_START_STREAM, 
				   ITS_RX_CMD_STOP_STREAM,
				   ITS_RX_CMD_CHANGE_RESOLUTION, 
				   ITS_RX_CMD_CHANGE_PHY,
				   ITS_RX_CMD_SEND_BLE_PARAMS};

struct its_rx_cb_evt_t {
	enum its_rx_cmd_e command;
	uint8_t *data;
	uint32_t len;
};

/** @brief Callback type for when an LED state change is received. */
typedef void (*its_rx_cb_t)(struct its_rx_cb_evt_t *evt);

/** @brief Callback struct used by the ITS Service. */
struct bt_its_cb {
	/** ITS RX callback. */
	its_rx_cb_t rx_cb;
};

enum its_img_info_data_type_e {ITS_IMG_INFO_DATA_TYPE_IMG_INFO = 1, ITS_IMG_INFO_DATA_TYPE_BLE_PARAMS = 2};

struct its_img_info_t {
    uint32_t file_size_bytes;   
};

struct its_ble_params_info_t {
    uint16_t mtu;
    uint16_t con_interval;
    uint8_t  tx_phy;
    uint8_t  rx_phy;
};

int bt_its_init(struct bt_its_cb *callbacks);

int bt_its_send_img_data(struct bt_conn *conn, uint8_t *buf, uint16_t length, uint16_t le_max_length);

int bt_its_send_img_info(struct its_img_info_t *img_info);

int bt_its_send_ble_params_info(struct its_ble_params_info_t *ble_params_info);

#ifdef __cplusplus
}
#endif

#endif /* BT_ITS_H_ */
