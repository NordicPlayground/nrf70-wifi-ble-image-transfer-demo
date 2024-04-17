/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include "socket_util.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(socket_util, CONFIG_LOG_DEFAULT_LEVEL);

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/net/socket.h>
/* Macro called upon a fatal error, reboots the device. */
#include <zephyr/sys/reboot.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log_ctrl.h>
#define FATAL_ERROR()                              \
	LOG_ERR("Fatal error! Rebooting the device."); \
	LOG_PANIC();                                   \
	IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

/* size of stack area used by each thread */
#define STACKSIZE 4096
/* scheduling priority used by each thread */
#define PRIORITY 3
/* Same as COMMAND_MAX_SIZE*/
#define BUFFER_MAX_SIZE 6

#define cam_port 60000
#define pc_port  60006

/**********External resources**************/
extern int wifi_softap_mode_ready(void);
extern int wifi_station_mode_ready(void);

int cam_socket;
struct sockaddr_in cam_addr;

struct sockaddr_in pc_addr;
socklen_t pc_addr_len=sizeof(pc_addr);

/* Define the semaphore wifi_net_ready */
struct k_sem wifi_net_ready;

uint8_t udp_head_and_tail[] = {0xff, 0xaa, 0x00, 0xff, 0xbb};
uint8_t udp_recv_buf[BUFFER_MAX_SIZE];
K_MSGQ_DEFINE(udp_recv_queue, sizeof(udp_recv_buf), 1, 4);

static net_util_udp_rx_callback_t udp_rx_cb = 0; 

void net_util_set_callback(net_util_udp_rx_callback_t udp_rx_callback)
{
	udp_rx_cb = udp_rx_callback;

	// If any messages are waiting in the queue, forward them immediately
	uint8_t buf[6];
	while (k_msgq_get(&udp_recv_queue, buf, K_NO_WAIT) == 0) {
		udp_rx_cb(buf, 6);
	}
}

uint8_t process_udp_rx_buffer(char *udp_rx_buf, char *command_buf)
{
	uint8_t command_length = 0;

	// Check if udp_rx_buf contains start code (0x55)
	if (udp_rx_buf[0] == 0x55)
	{
		// Find the end code (0xAA) and extract the command
		for (uint8_t i = 1; i < UDP_COMMAND_MAX_SIZE; i++)
		{
			if (udp_rx_buf[i] == 0xAA)
			{
				// Copy the command to command_buf
				for (uint8_t j = 1; j < i; j++)
				{
					command_buf[j - 1] = udp_rx_buf[j];
				}
				command_length = i - 1; // Length of the command
				break;
			}
		}
	}
	return command_length;
}

static void trigger_rx_udp_callback_if_set(uint8_t *msg)
{
	if (udp_rx_cb != 0) {
		udp_rx_cb(udp_recv_buf, 6);
	} else {
		k_msgq_put(&udp_recv_queue, &udp_recv_buf, K_NO_WAIT);
	}
}

/* Thread to setup WiFi, Network, Sockets step by step */
static void wifi_net_sockets(void)
{
	int ret;	

	k_sem_init(&wifi_net_ready, 0, 1);

	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}
 
    if(true){
		ret = wifi_station_mode_ready();
	}else{
		ret = wifi_softap_mode_ready();
	}

	if (ret < 0)
	{
		LOG_ERR("wifi network connection is not ready, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	cam_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (cam_socket < 0)
	{
		LOG_ERR("Failed to create UDP socket: %d", -errno);
		FATAL_ERROR();
		return;
	}

	cam_addr.sin_family = AF_INET;
	cam_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	cam_addr.sin_port = htons(cam_port);
	ret = bind(cam_socket, (struct sockaddr *)&cam_addr, sizeof(cam_addr));
	if (ret < 0)
	{
		LOG_ERR("bind, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	pc_addr.sin_family = AF_INET;
	pc_addr.sin_addr.s_addr = htonl(0xC0A80102);//192.168.1.2
	pc_addr.sin_port = htons(pc_port);
	ret = connect(cam_socket, (struct sockaddr *)&pc_addr, sizeof(struct sockaddr_in));
	if (ret < 0)
	{
		LOG_ERR("connect, error: %d", -errno);
		FATAL_ERROR();
		return;
	}
	LOG_INF("Sockets are ready!");

	while (1)
	{
		ret = recvfrom(cam_socket, udp_recv_buf, sizeof(udp_recv_buf), 0, (struct sockaddr *)&pc_addr, &pc_addr_len);
		if (ret < 0)
		{
			LOG_ERR("[%d] Cannot receive udp message (%d)", ret,
					-errno);
			return;
		}
		trigger_rx_udp_callback_if_set(udp_recv_buf);
		k_yield();
	}
}

K_THREAD_DEFINE(wifi_net_sockets_id, STACKSIZE, wifi_net_sockets, NULL, NULL, NULL,
				PRIORITY, 0, 0);