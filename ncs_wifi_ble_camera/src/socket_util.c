/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
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
#include "socket_util.h"

#define FATAL_ERROR()                              \
	LOG_ERR("Fatal error! Rebooting the device."); \
	LOG_PANIC();                                   \
	IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

/* size of stack area used by each thread */
#define STACKSIZE 8192
/* scheduling priority used by each thread */
#define PRIORITY 3
/* Same as COMMAND_MAX_SIZE*/
#define BUFFER_MAX_SIZE 6

#define pc_port  60000
#define cam_udp_port 60010 // use for either udp or tcp server
#define cam_port 60010


/**********External Resources START**************/
extern int wifi_softap_mode_ready(void);
extern int wifi_station_mode_ready(void);
/**********External Resources END**************/

int cam_socket;
int pc_socket;
int cam_udp_socket;
int cam_tcp_server_socket;
struct sockaddr_in cam_udp_addr;
struct sockaddr_in cam_tcp_server_addr;

struct sockaddr_in pc_addr;
socklen_t pc_addr_len=sizeof(pc_addr);

/* Define the semaphore wifi_net_ready */
struct k_sem wifi_net_ready;
enum wifi_modes wifi_mode=WIFI_SOFTAP_MODE;

uint8_t socket_recv_buf[BUFFER_MAX_SIZE];
K_MSGQ_DEFINE(socket_recv_queue, sizeof(socket_recv_buf), 1, 4);

static net_util_socket_rx_callback_t socket_rx_cb = 0; 

void cam_send(const void *buf, size_t len){
      //sendto(cam_socket, buf, len, 0, (struct sockaddr *)&pc_addr,  sizeof(pc_addr));
	  if (send(pc_socket, buf, len, 0) == -1) {
            perror("Sending failed");
            close(cam_socket);
			close(pc_socket);
			FATAL_ERROR();
			return;
        }
	  //LOG_HEXDUMP_INF(buf, len, "SocektSend");
}

void net_util_set_callback(net_util_socket_rx_callback_t socket_rx_callback)
{
	socket_rx_cb = socket_rx_callback;

	// If any messages are waiting in the queue, forward them immediately
	uint8_t buf[6];
	while (k_msgq_get(&socket_recv_queue, buf, K_NO_WAIT) == 0) {
		socket_rx_cb(buf, 6);
	}
}

uint8_t process_socket_rx_buffer(char *socket_rx_buf, char *command_buf)
{
	uint8_t command_length = 0;

	// Check if socket_rx_buf contains start code (0x55)
	if (socket_rx_buf[0] == 0x55)
	{
		// Find the end code (0xAA) and extract the command
		for (uint8_t i = 1; i < CAM_COMMAND_MAX_SIZE; i++)
		{
			if (socket_rx_buf[i] == 0xAA)
			{
				// Copy the command to command_buf
				for (uint8_t j = 1; j < i; j++)
				{
					command_buf[j - 1] = socket_rx_buf[j];
				}
				command_length = i - 1; // Length of the command
				break;
			}
		}
	}
	return command_length;
}

static void trigger_socket_rx_callback_if_set()
{
	if (socket_rx_cb != 0) {
		socket_rx_cb(socket_recv_buf, 6);
	} else {
		k_msgq_put(&socket_recv_queue, &socket_recv_buf, K_NO_WAIT);
	}
}

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	if (buttons & DK_BTN1_MSK) {
		wifi_mode = WIFI_STATION_MODE;
	}
}

/* Thread to setup WiFi, Network, Sockets step by step */
static void wifi_net_sockets(void)
{
	int ret;
	char pc_addr_str[32];
	
	cam_udp_addr.sin_family = AF_INET;
	cam_udp_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	cam_udp_addr.sin_port = htons(cam_udp_port);

	cam_tcp_server_addr.sin_family = AF_INET;
	cam_tcp_server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	cam_tcp_server_addr.sin_port = htons(cam_port);

	k_sem_init(&wifi_net_ready, 0, 1);

	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}
	if (dk_buttons_init(button_handler)!= 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}
	
	LOG_INF("\r\n\r\nPress Button1 in THREE seconds to change WiFi from softAP mode to Station mode.\r\n");
	k_sleep(K_SECONDS(3));

    if(wifi_mode == WIFI_SOFTAP_MODE){
		ret = wifi_softap_mode_ready();
	}else{
		ret = wifi_station_mode_ready();
	}

	if (ret < 0)
	{
		LOG_ERR("wifi network connection is not ready, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	/*************TCP Socket START**************/
	cam_tcp_server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (cam_tcp_server_socket < 0)
	{
		LOG_ERR("Failed to create socket: %d", -errno);
		FATAL_ERROR();
		return;
	}
    
	ret = bind(cam_tcp_server_socket, (struct sockaddr *)&cam_tcp_server_addr, sizeof(cam_tcp_server_addr));
	if (ret < 0)
	{
		LOG_ERR("bind, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	ret = listen(cam_tcp_server_socket, 5);
	if (ret < 0)
	{
		LOG_ERR("listen, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	pc_socket = accept(cam_tcp_server_socket, (struct sockaddr *)&pc_addr, &pc_addr_len);
	if (pc_socket < 0)
	{
		LOG_ERR("accept, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	// ret = close(cam_tcp_server_socket);
	// if (ret < 0)
	// {
	// 	LOG_ERR("close, error: %d", -errno);
	// 	FATAL_ERROR();
	// 	return;
	// }
	/*************TCP Socket END**************/

	/*************UDP Socket START**************/
	// cam_udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	// if (cam_udp_socket < 0)
	// {
	// 	LOG_ERR("Failed to create socket: %d", -errno);
	// 	FATAL_ERROR();
	// 	return;
	// }
	// ret = bind(cam_udp_socket, (struct sockaddr *)&cam_udp_addr, sizeof(cam_udp_addr));
	// if (ret < 0)
	// {
	// 	LOG_ERR("bind, error: %d", -errno);
	// 	FATAL_ERROR();
	// 	return;
	// }
	// ret = recvfrom(cam_udp_socket, socket_recv_buf, sizeof(socket_recv_buf), 0, (struct sockaddr *)&pc_addr, &pc_addr_len);
	// if (ret < 0)
	// {
	// 	LOG_ERR("recvfrom, error: %d", -errno);
	// 	FATAL_ERROR();
	// 	return;
	// }
	/*************UDP Socket END**************/
	inet_ntop(pc_addr.sin_family, &pc_addr.sin_addr, pc_addr_str, sizeof(pc_addr_str));
	LOG_INF("Connect with PC through WiFi, PC IPAddr = %s, Port = %d\n", pc_addr_str, ntohs(pc_addr.sin_port));

	LOG_INF("Socket is ready!");
	cam_socket = cam_tcp_server_socket;

// #define BUFFER_SIZE 1024
// char buffer[BUFFER_SIZE];
// 	 // Receive and send data
//     while (1) {
//         //int bytes_received = recvfrom(pc_socket, buffer, BUFFER_SIZE, (struct sockaddr *)&pc_addr, &pc_addr_len);
// 		int bytes_received = recv(pc_socket, buffer, BUFFER_SIZE,0);
//         if (bytes_received == -1) {
//             LOG_ERR("Receiving failed");
//             close(cam_socket);
// 			close(pc_socket);
// 			FATAL_ERROR();
// 			return;
//         } else if (bytes_received == 0) {
//             LOG_INF("Client disconnected.\n");
//             break;
//         }
//         //buffer[bytes_received] = '\0';
// 		LOG_INF("bytes_received:%d", bytes_received);
// 		LOG_HEXDUMP_INF(buffer, sizeof(bytes_received), "hexdump");


//         // Echo back to client
// 		//sendto(cam_socket, buf, len, 0, (struct sockaddr *)&pc_addr,  sizeof(pc_addr));
//         if (send(pc_socket, buffer, bytes_received, 0) == -1) {
//             perror("Sending failed");
//             close(cam_socket);
// 			close(pc_socket);
// 			FATAL_ERROR();
// 			return;
//         }
//     }

	trigger_socket_rx_callback_if_set(socket_recv_buf);

	while (1)
	{
		//ret = recvfrom(cam_udp_socket, socket_recv_buf, sizeof(socket_recv_buf), 0, (struct sockaddr *)&pc_addr, &pc_addr_len)
		// if (ret < 0)
		// {
		// 	LOG_ERR("[%d] Cannot receive message (%d)", ret,
		// 			-errno);
		// 	return;
		// }
		int bytes_received = recv(pc_socket, socket_recv_buf, sizeof(socket_recv_buf),0);
        if (bytes_received == -1) {
            LOG_ERR("Receiving failed");
            close(cam_socket);
			close(pc_socket);
			FATAL_ERROR();
			return;
        } else if (bytes_received == 0) {
            LOG_INF("Client disconnected.\n");
            break;
        }
		trigger_socket_rx_callback_if_set(socket_recv_buf);
		k_yield();
	}
}

K_THREAD_DEFINE(wifi_net_sockets_id, STACKSIZE, wifi_net_sockets, NULL, NULL, NULL,
				PRIORITY, 0, 0);