#ifndef __NET_UTIL_H
#define __NET_UTIL_H

#include <zephyr/kernel.h>

typedef void (*net_util_udp_rx_callback_t)(uint8_t *data, uint16_t len);

void net_util_set_callback(net_util_udp_rx_callback_t udp_rx_callback);

#endif