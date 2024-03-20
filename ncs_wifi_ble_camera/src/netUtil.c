/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */
#include "net_util.h"
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(NetUtil, CONFIG_LOG_DEFAULT_LEVEL);

#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
/* Macro called upon a fatal error, reboots the device. */
#include <zephyr/sys/reboot.h>

/* Include the necessary header files */
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/socket.h>
#include <net/wifi_mgmt_ext.h>

/* Include the header file for the Wi-FI credentials library */
#include <net/wifi_credentials.h>

#include <dk_buttons_and_leds.h>

#include <zephyr/logging/log_ctrl.h>
#define FATAL_ERROR()                              \
	LOG_ERR("Fatal error! Rebooting the device."); \
	LOG_PANIC();                                   \
	IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

/* size of stack area used by each thread */
#define STACKSIZE 4096
/* scheduling priority used by each thread */
#define PRIORITY 7

/* Same as COMMAND_MAX_SIZE*/
#define BUFFER_MAX_SIZE 6

/* Define a macro for the relevant network events */
#define L2_EVENT_MASK (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)
#define L3_EVENT_MASK NET_EVENT_IPV4_DHCP_BOUND

/* Declare the callback structure for Wi-Fi events */
static struct net_mgmt_event_callback wifi_mgmt_cb;
static struct net_mgmt_event_callback net_mgmt_cb;

/* Define the boolean wifi_connected and the semaphore wifi_net_ready */
static bool wifi_connected;
static K_SEM_DEFINE(wifi_net_ready, 0, 1)

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

/*
** Two sockets are use by UDP Server(WiFiCam+nRF7002DK)
** UDP Client(WiFiCamHost+PC):50000 <- UDP Server(WiFiCam+nRF7002DK):60000 = socket_recv
** UDP Client(WiFiCamHost+PC):50005 -> UDP Server(WiFiCam+nRF7002DK):60006 = socket_send
** socket_recv is built by UDP server to wait for UDP client connet to its address, then server can know client address.
** the server build a new socket_send to send WiFiCam data like camera info, video frame to client. The previous socket_recv is
** used to recive command from UDP client.
*/
#define server_recv_port 60000
#define server_send_port 60006
#define client_recv_port 50000
#define client_send_port 50005

int socket_send;
struct sockaddr_in server_addr;

static int cmd_wifi_status(void)
{
	struct net_if *iface = net_if_get_default();
	struct wifi_iface_status status = {0};

	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
				 sizeof(struct wifi_iface_status)))
	{
		LOG_INF("Status request failed");

		return -ENOEXEC;
	}

	LOG_INF("==================");
	LOG_INF("WiFi State: %s", wifi_state_txt(status.state));

	if (status.state >= WIFI_STATE_ASSOCIATED)
	{
		LOG_INF("Interface Mode: %s",
				wifi_mode_txt(status.iface_mode));
		LOG_INF("Link Mode: %s",
				wifi_link_mode_txt(status.link_mode));
		LOG_INF("SSID: %-32s", status.ssid);
		LOG_INF("Band: %s", wifi_band_txt(status.band));
		LOG_INF("Channel: %d", status.channel);
		LOG_INF("Security: %s", wifi_security_txt(status.security));
		LOG_INF("MFP: %s", wifi_mfp_txt(status.mfp));
		LOG_INF("RSSI: %d", status.rssi);
	}
	return 0;
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
									uint32_t mgmt_event, struct net_if *iface)
{
	switch (mgmt_event)
	{
	case NET_EVENT_WIFI_CONNECT_RESULT:
		LOG_INF("WiFi connected");
		wifi_connected = true;
		break;
	case NET_EVENT_WIFI_DISCONNECT_RESULT:
		if (wifi_connected == false)
		{
			LOG_INF("Waiting for WiFi to be wifi_connected");
		}
		else
		{
			dk_set_led_off(DK_LED1);
			LOG_INF("WiFi disconnected");
			wifi_connected = false;
		}
		k_sem_reset(&wifi_net_ready);
		break;
	default:
		break;
	}
	cmd_wifi_status();
}

static void on_net_event_dhcp_bound(struct net_mgmt_event_callback *cb)
{
	/* Get DHCP info from struct net_if_dhcpv4 and print */
	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
	const struct in_addr *addr = &dhcpv4->requested_ip;
	char dhcp_info[128];
	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

	LOG_INF("WiFi Camera Server is ready on nRF7002DK, copy and paste %s:%d in Target WiFi Camera Address window on WiFi Camera Host.", dhcp_info, server_recv_port);
}

/* Define the callback function for network events */
static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
								   uint32_t mgmt_event, struct net_if *iface)
{
	if ((mgmt_event & L3_EVENT_MASK) != mgmt_event)
	{
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_DHCP_BOUND)
	{
		LOG_INF("Network DHCP bound");
		on_net_event_dhcp_bound(cb);
		dk_set_led_on(DK_LED1);
		k_sem_give(&wifi_net_ready);
		return;
	}
}

#if defined(CONFIG_WIFI_CREDENTIALS_STATIC)
/* Define the function to populate the Wi-Fi credential parameters */
static int wifi_args_to_params(struct wifi_connect_req_params *params)
{

	/* Populate the SSID and password */
	params->ssid = CONFIG_WIFI_CREDENTIALS_STATIC_SSID;
	params->ssid_length = strlen(params->ssid);

	params->psk = CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD;
	params->psk_length = strlen(params->psk);

	/* Populate the rest of the relevant members */
	params->channel = WIFI_CHANNEL_ANY;
	params->security = WIFI_SECURITY_TYPE_PSK;
	params->mfp = WIFI_MFP_OPTIONAL;
	params->timeout = SYS_FOREVER_MS;

	return 0;
}
#endif

static int wifi_network_ready(void)
{
#if defined(CONFIG_WIFI_CREDENTIALS_STATIC)
	/* Declare the variable for the network configuration parameters */
	struct wifi_connect_req_params cnx_params;

	/* Get the network interface */
	struct net_if *iface = net_if_get_first_wifi();
	if (iface == NULL)
	{
		LOG_ERR("Returned network interface is NULL");
		return -1;
	}
#endif

	if (dk_leds_init() != 0)
	{
		LOG_ERR("Failed to initialize the LED library");
	}

	/* Sleep to allow initialization of Wi-Fi driver */
	k_sleep(K_SECONDS(1));

	/* Initialize and add the callback function for network events */
	net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler, L2_EVENT_MASK);
	net_mgmt_add_event_callback(&wifi_mgmt_cb);

	net_mgmt_init_event_callback(&net_mgmt_cb, net_mgmt_event_handler, L3_EVENT_MASK);
	net_mgmt_add_event_callback(&net_mgmt_cb);

#if defined(CONFIG_WIFI_CREDENTIALS_STATIC)
	/* Populate cnx_params with the network configuration */
	wifi_args_to_params(&cnx_params);

	/* Call net_mgmt() to request the Wi-Fi connection */
	LOG_INF("Connecting to Wi-Fi with static crendentials.");
	int err = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params, sizeof(struct wifi_connect_req_params));
	if (err)
	{
		LOG_ERR("Connecting to Wi-Fi failed, err: %d", err);
		return ENOEXEC;
	}
#endif

	k_sem_take(&wifi_net_ready, K_FOREVER);

	return 0;
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
	static int socket_recv;
	struct sockaddr_in client_addr;
	socklen_t client_addr_len;
	char addr_str[32];

	ret = wifi_network_ready();
	if (ret < 0)
	{
		LOG_ERR("wifi network connection is not ready, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	socket_recv = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_recv < 0)
	{
		LOG_ERR("Failed to create UDP socket: %d", -errno);
		FATAL_ERROR();
		return;
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(server_recv_port);

	ret = bind(socket_recv, (struct sockaddr *)&server_addr, sizeof(server_addr));
	if (ret < 0)
	{
		LOG_ERR("bind, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	ret = recvfrom(socket_recv, udp_recv_buf, sizeof(udp_recv_buf), 0, (struct sockaddr *)&client_addr, &client_addr_len);
	if (ret < 0)
	{
		LOG_ERR("recvfrom, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	trigger_rx_udp_callback_if_set(udp_recv_buf);

	inet_ntop(client_addr.sin_family, &client_addr.sin_addr, addr_str, sizeof(addr_str));
	LOG_INF("UDP Client(WiFiCamHost) IPAddr = %s, Port = %d\n", addr_str, ntohs(client_addr.sin_port));

	socket_send = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_send < 0)
	{
		LOG_ERR("Failed to create UDP socket: %d", -errno);
		FATAL_ERROR();
		return;
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(server_send_port);
	ret = bind(socket_send, (struct sockaddr *)&server_addr, sizeof(server_addr));
	if (ret < 0)
	{
		LOG_ERR("bind, error: %d", -errno);
		FATAL_ERROR();
		return;
	}

	client_addr.sin_port = htons(client_send_port);
	ret = connect(socket_send, (struct sockaddr *)&client_addr, sizeof(struct sockaddr_in));
	if (ret < 0)
	{
		LOG_ERR("connect, error: %d", -errno);
		FATAL_ERROR();
		return;
	}
	LOG_INF("Sockets are ready!");

	while (1)
	{
		ret = recvfrom(socket_recv, udp_recv_buf, sizeof(udp_recv_buf), 0, (struct sockaddr *)&client_addr, &client_addr_len);
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