#ifndef __APP_BLUETOOTH_H
#define __APP_BLUETOOTH_H

#include <zephyr/types.h>
#include <zephyr/kernel.h>

// Callbacks from the app_bluetooth module to the application
typedef void (*app_bt_take_picture_cb)(void);

struct app_bt_cb {
    app_bt_take_picture_cb take_picture;
};

int app_bt_init(const struct app_bt_cb *callbacks);

int app_bt_send_picture_header(uint32_t pic_size);

int app_bt_send_picture_data(uint8_t *buf, uint16_t len);

#endif