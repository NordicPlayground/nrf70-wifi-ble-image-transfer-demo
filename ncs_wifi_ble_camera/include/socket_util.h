#ifndef __SOCKET_UTIL_H
#define __SOCKET_UTIL_H

#include <zephyr/kernel.h>

#define CAM_COMMAND_MAX_SIZE 6

enum wifi_modes {
	WIFI_STATION_MODE = 0,
    WIFI_SOFTAP_MODE,
};

typedef void (*net_util_socket_rx_callback_t)(uint8_t *data, uint16_t len);

void net_util_set_callback(net_util_socket_rx_callback_t socket_rx_callback);
void cam_send(const void *buf, size_t len);

uint8_t process_socket_rx_buffer(char *udp_rx_buf, char *command_buf);

#endif