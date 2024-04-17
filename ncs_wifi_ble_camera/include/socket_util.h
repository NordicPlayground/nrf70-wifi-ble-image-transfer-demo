#ifndef __SOCKET_UTIL_H
#define __SOCKET_UTIL_H

#include <zephyr/kernel.h>

#define UDP_COMMAND_MAX_SIZE 6

/* actucal command not count start(0x55) and stop(0xAA) codes*/
extern uint8_t udp_head_and_tail[];

typedef void (*net_util_udp_rx_callback_t)(uint8_t *data, uint16_t len);

void net_util_set_callback(net_util_udp_rx_callback_t udp_rx_callback);

uint8_t process_udp_rx_buffer(char *udp_rx_buf, char *command_buf);

#endif