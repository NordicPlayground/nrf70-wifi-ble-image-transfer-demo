// /*
//  * Copyright (c) 2022 Nordic Semiconductor ASA
//  *
//  * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
//  */

// /** @file
//  * @brief WiFi station sample
//  */

// #include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(sta, CONFIG_LOG_DEFAULT_LEVEL);

// #include <nrfx_clock.h>
// #include <zephyr/kernel.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <zephyr/shell/shell.h>
// #include <zephyr/sys/printk.h>
// #include <zephyr/init.h>

// #include <zephyr/net/net_if.h>
// #include <zephyr/net/wifi_mgmt.h>
// #include <zephyr/net/net_event.h>
// #include <zephyr/drivers/gpio.h>
// #include <zephyr/net/socket.h>
// #include <zephyr/sys/reboot.h>
// #include <zephyr/logging/log.h>
// #include <zephyr/logging/log_ctrl.h>

// #include <qspi_if.h>

// #include "net_private.h"

// #define WIFI_SHELL_MODULE "wifi"

// #define WIFI_SHELL_MGMT_EVENTS (NET_EVENT_WIFI_CONNECT_RESULT |		\
// 				NET_EVENT_WIFI_DISCONNECT_RESULT)
// /* Macro called upon a fatal error, reboots the device. */
// #define FATAL_ERROR()					\
// 	LOG_ERR("Fatal error! Rebooting the device.");	\
// 	LOG_PANIC();					\
// 	IF_ENABLED(CONFIG_REBOOT, (sys_reboot(0)))

// #define UDP_IP_HEADER_SIZE 28

// #define MAX_SSID_LEN        32
// #define STATUS_POLLING_MS   300

// /* 1000 msec = 1 sec */
// #define LED_SLEEP_TIME_MS   100

// /* The devicetree node identifier for the "led0" alias. */
// #define LED0_NODE DT_ALIAS(led0)
// /*
//  * A build error on this line means your board is unsupported.
//  * See the sample documentation for information on how to fix this.
//  */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

// static struct net_mgmt_event_callback wifi_shell_mgmt_cb;
// static struct net_mgmt_event_callback net_shell_mgmt_cb;

// /* Variables used to perform socket operations. */
// static int fd;

// /* Arducam mega commands https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/HostCommunicationProtocol.html*/
// #define COMMAND_MAX_SIZE 6 
// uint8_t udp_recv_buf[COMMAND_MAX_SIZE];
// uint8_t udp_recive_buf[COMMAND_MAX_SIZE];
// /* actucal command not count start(0x55) and stop(0xAA) codes*/
// uint8_t command_buf[COMMAND_MAX_SIZE-2];

// /* Forward declarations */
// static void server_transmission_work_fn(struct k_work *work);
// static void video_preview_work_fn(struct k_work *work);

// /* Work item used to schedule periodic UDP transmissions. */
// static K_WORK_DELAYABLE_DEFINE(server_transmission_work, server_transmission_work_fn);
// static K_WORK_DELAYABLE_DEFINE(video_preview_work, video_preview_work_fn);
// K_MSGQ_DEFINE(addr_queue, sizeof(struct sockaddr_in), 1, 4);
// K_MSGQ_DEFINE(udp_recv_queue, sizeof(udp_recv_buf), 1, 4);
// K_SEM_DEFINE(net_connected_sem, 0, 1);

// static struct {
// 	const struct shell *sh;
// 	union {
// 		struct {
// 			uint8_t connected	: 1;
// 			uint8_t connect_result	: 1;
// 			uint8_t disconnect_requested	: 1;
// 			uint8_t _unused		: 5;
// 		};
// 		uint8_t all;
// 	};
// } context;



// /*****************Ardum Mega Camera Utilities***************************/
// #include <zephyr/drivers/video.h>
// #include <zephyr/drivers/video/arducam_mega.h>
// #include <zephyr/device.h>

// static uint8_t head_and_tail[] = {0xff, 0xaa, 0x00, 0xff, 0xbb};

// #define RESET_CAMERA             0XFF
// #define SET_PICTURE_RESOLUTION   0X01
// #define SET_VIDEO_RESOLUTION     0X02
// #define SET_BRIGHTNESS           0X03
// #define SET_CONTRAST             0X04
// #define SET_SATURATION           0X05
// #define SET_EV                   0X06
// #define SET_WHITEBALANCE         0X07
// #define SET_SPECIAL_EFFECTS      0X08
// #define SET_FOCUS_ENABLE         0X09
// #define SET_EXPOSURE_GAIN_ENABLE 0X0A
// #define SET_WHITE_BALANCE_ENABLE 0X0C
// #define SET_MANUAL_GAIN          0X0D
// #define SET_MANUAL_EXPOSURE      0X0E
// #define GET_CAMERA_INFO          0X0F
// #define TAKE_PICTURE             0X10
// #define SET_SHARPNESS            0X11
// #define DEBUG_WRITE_REGISTER     0X12
// #define STOP_STREAM              0X21
// #define GET_FRM_VER_INFO         0X30
// #define GET_SDK_VER_INFO         0X40
// #define SET_IMAGE_QUALITY        0X50
// #define SET_LOWPOWER_MODE        0X60

// const struct device *video;
// struct video_buffer *vbuf;

// volatile uint8_t preview_on;
// volatile uint8_t capture_flag;

// const uint32_t pixel_format_table[] = {
// 	VIDEO_PIX_FMT_JPEG,
// 	VIDEO_PIX_FMT_RGB565,
// 	VIDEO_PIX_FMT_YUYV,
// };

// const uint16_t resolution_table[][2] = {
// 	{160, 120},  {320, 240},   {640, 480},   {800, 600},   {1280, 720},
// 	{1280, 960}, {1600, 1200}, {1920, 1080}, {2048, 1536}, {2592, 1944},
// 	{96, 96},    {128, 128},   {320, 320},
// };

// const uint8_t resolution_num = sizeof(resolution_table) / 4;

// static uint8_t current_resolution;
// static uint8_t target_resolution;
// static uint8_t take_picture_fmt = 0x1a;

// void cam_to_host_command_send(uint8_t type, uint8_t *buffer, uint32_t length)
// {
// 	head_and_tail[2] = type;
// 	send(fd,  &head_and_tail[0], 3, 0);
// 	send(fd,  (uint8_t *)&length, 4, 0);
// 	send(fd,  buffer, length, 0);
// 	send(fd,  &head_and_tail[3], 2, 0);
// }

// int set_mega_resolution(uint8_t sfmt)
// {
// 	uint8_t resolution = sfmt & 0x0f;
// 	uint8_t pixelformat = (sfmt & 0x70) >> 4;

// 	if (resolution > resolution_num || pixelformat > 3) {
// 		return -1;
// 	}
// 	struct video_format fmt = {.width = resolution_table[resolution][0],
// 				   .height = resolution_table[resolution][1],
// 				   .pixelformat = pixel_format_table[pixelformat - 1]};
// 	current_resolution = resolution;
// 	return video_set_format(video, VIDEO_EP_OUT, &fmt);
// }

// int take_picture(void)
// {
// 	int err;
// 	enum video_frame_fragmented_status f_status;

// 	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
// 	if (err) {
// 		LOG_ERR("Unable to dequeue video buf");
// 		return -1;
// 	}

// 	f_status = vbuf->flags;

// 	head_and_tail[2] = 0x01;
// 	send(fd, &head_and_tail[0], 3, 0);
// 	send(fd, (uint8_t *)&vbuf->bytesframe, 4, 0);

// 	target_resolution=(((current_resolution & 0x0f) << 4)|0x01);
// 	send(fd, &target_resolution, 1, 0);
	
// 	send(fd, vbuf->buffer, vbuf->bytesused, 0);

// 	video_enqueue(video, VIDEO_EP_OUT, vbuf);
// 	while (f_status == VIDEO_BUF_FRAG) {
// 		video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
// 		f_status = vbuf->flags;
// 		send(fd, vbuf->buffer, vbuf->bytesused, 0);
// 		video_enqueue(video, VIDEO_EP_OUT, vbuf);
// 	}
// 	send(fd, &head_and_tail[3], 2, 0);

// 	return 0;
// }

// void video_preview(void)
// {
// 	int err;
// 	enum video_frame_fragmented_status f_status;
// 	if (!preview_on) {
// 		return;
// 	}
// 	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
// 	if (err) {
// 		LOG_INF("Unable to dequeue video buf");
// 		return;
// 	}
// 	f_status = vbuf->flags;
// 	LOG_INF("f_status%d",f_status);
// 	if (capture_flag == 1) {
// 		capture_flag = 0;
// 		head_and_tail[2] = 0x01;
// 		send(fd, &head_and_tail[0], 3, 0);
// 		send(fd, (uint8_t *)&vbuf->bytesframe, 4, 0);
// 		target_resolution=(((current_resolution & 0x0f) << 4)|0x01);
// 		send(fd, &target_resolution, 1, 0);
// 	}

// 	send(fd, vbuf->buffer, vbuf->bytesused, 0);

// 	if (f_status == VIDEO_BUF_EOF) {
// 		send(fd, &head_and_tail[3], 2, 0);
// 		capture_flag = 1;
// 	}

// 	err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
// 	if (err) {
// 		LOG_ERR("Unable to requeue video buf");
// 		return;
// 	}
// }

// int report_mega_info(void)
// {
// 	char str_buf[400];
// 	uint32_t str_len;
// 	char *mega_type;
// 	struct arducam_mega_info mega_info;

// 	video_get_ctrl(video, VIDEO_CID_ARDUCAM_INFO, &mega_info);

// 	switch (mega_info.camera_id) {
// 	case ARDUCAM_SENSOR_3MP_1:
// 	case ARDUCAM_SENSOR_3MP_2:
// 		mega_type = "3MP";
// 		break;
// 	case ARDUCAM_SENSOR_5MP_1:
// 		mega_type = "5MP";
// 		break;
// 	case ARDUCAM_SENSOR_5MP_2:
// 		mega_type = "5MP_2";
// 		break;
// 	default:
// 		return -ENODEV;
// 	}

// 	sprintf(str_buf,
// 		"ReportCameraInfo\r\nCamera Type:%s\r\n"
// 		"Camera Support Resolution:%d\r\nCamera Support "
// 		"special effects:%d\r\nCamera Support Focus:%d\r\n"
// 		"Camera Exposure Value Max:%ld\r\nCamera Exposure Value "
// 		"Min:%d\r\nCamera Gain Value Max:%d\r\nCamera Gain Value "
// 		"Min:%d\r\nCamera Support Sharpness:%d\r\n",
// 		mega_type, mega_info.support_resolution, mega_info.support_special_effects,
// 		mega_info.enable_focus, mega_info.exposure_value_max, mega_info.exposure_value_min,
// 		mega_info.gain_value_max, mega_info.gain_value_min, mega_info.enable_sharpness);
// 	LOG_INF("%s",str_buf);
// 	str_len = strlen(str_buf);
// 	cam_to_host_command_send(0x02, str_buf, str_len);
// 	return 0;
// }

// uint8_t recv_process(uint8_t *buff)
// {
// 	switch (buff[0]) {
// 	case SET_PICTURE_RESOLUTION:
// 		if (set_mega_resolution(buff[1]) == 0) {
// 			take_picture_fmt = buff[1];
// 		}
// 		break;
// 	case SET_VIDEO_RESOLUTION:
// 		LOG_INF("SET_VIDEO_RESOLUTION");
// 		if (preview_on == 0) {
// 			LOG_INF("SET_VIDEO_RESOLUTION preview_on");
// 			set_mega_resolution(buff[1] | 0x10);
// 			LOG_INF("video_stream_start");
// 			video_stream_start(video);
// 			//(void)k_work_schedule(&video_preview_work,K_NO_WAIT);
// 			capture_flag = 1;
// 		}
// 		preview_on = 1;
// 		break;
// 	case SET_BRIGHTNESS:
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_BRIGHTNESS, &buff[1]);
// 		break;
// 	case SET_CONTRAST:
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_CONTRAST, &buff[1]);
// 		break;
// 	case SET_SATURATION:
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_SATURATION, &buff[1]);
// 		break;
// 	case SET_EV:
// 		video_set_ctrl(video, VIDEO_CID_ARDUCAM_EV, &buff[1]);
// 		break;
// 	case SET_WHITEBALANCE:
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_WHITE_BAL, &buff[1]);
// 		break;
// 	case SET_SPECIAL_EFFECTS:
// 		video_set_ctrl(video, VIDEO_CID_ARDUCAM_COLOR_FX, &buff[1]);
// 		break;
// 	case SET_EXPOSURE_GAIN_ENABLE:
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_EXPOSURE_AUTO, &buff[1]);
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_GAIN_AUTO, &buff[1]);
// 	case SET_WHITE_BALANCE_ENABLE:
// 		video_set_ctrl(video, VIDEO_CID_CAMERA_WHITE_BAL_AUTO, &buff[1]);
// 		break;
// 	case SET_SHARPNESS:
// 		video_set_ctrl(video, VIDEO_CID_ARDUCAM_SHARPNESS, &buff[1]);
// 		break;
// 	case SET_MANUAL_GAIN:
// 		uint16_t gain_value = (buff[1] << 8) | buff[2];

// 		video_set_ctrl(video, VIDEO_CID_CAMERA_GAIN, &gain_value);
// 		break;
// 	case SET_MANUAL_EXPOSURE:
// 		uint32_t exposure_value = (buff[1] << 16) | (buff[2] << 8) | buff[3];

// 		video_set_ctrl(video, VIDEO_CID_CAMERA_EXPOSURE, &exposure_value);
// 		break;
// 	case GET_CAMERA_INFO:
// 		report_mega_info();
// 		break;
// 	case TAKE_PICTURE:
// 		video_stream_start(video);
// 		take_picture();
// 		video_stream_stop(video);
// 		break;
// 	case STOP_STREAM:
// 		if (preview_on) {
// 			send(fd, &head_and_tail[3], 2, 0);
// 			video_stream_stop(video);
// 			set_mega_resolution(take_picture_fmt);
// 		}
// 		preview_on = 0;
// 		break;
// 	case RESET_CAMERA:
// 		video_set_ctrl(video, VIDEO_CID_ARDUCAM_RESET, NULL);
// 		break;
// 	case SET_IMAGE_QUALITY:
// 		video_set_ctrl(video, VIDEO_CID_JPEG_COMPRESSION_QUALITY, &buff[1]);
// 		break;
// 	case SET_LOWPOWER_MODE:
// 		video_set_ctrl(video, VIDEO_CID_ARDUCAM_LOWPOWER, &buff[1]);
// 		break;
// 	default:
// 		break;
// 	}

// 	return buff[0];
// }

// uint8_t process_udp_rx_buffer(char *udp_rx_buf, char *command_buf) {
//     uint8_t command_length = 0;

//     // Check if udp_rx_buf contains start code (0x55)
//     if (udp_rx_buf[0] == 0x55) {
//         // Find the end code (0xAA) and extract the command
//         for (uint8_t i = 1; i < COMMAND_MAX_SIZE; i++) {
//             if (udp_rx_buf[i] == 0xAA) {
//                 // Copy the command to command_buf
//                 for (uint8_t j = 1; j < i; j++) {
//                     command_buf[j - 1] = udp_rx_buf[j];
//                 }
//                 command_length = i - 1; // Length of the command
//                 break;
//             }
//         }
//     }

//     return command_length;
// }

// int count=0;
// static void server_transmission_work_fn(struct k_work *work)
// {
// 	int err;
//         count++;

// 	char buffer[CONFIG_UDP_SAMPLE_DATA_UPLOAD_SIZE_BYTES] = {"GetIn"};

// 	LOG_INF("Transmitting UDP/IP payload of %d bytes to the "
// 		"IP address %s, port number %d",
// 		CONFIG_UDP_SAMPLE_DATA_UPLOAD_SIZE_BYTES + UDP_IP_HEADER_SIZE,
// 		CONFIG_UDP_SAMPLE_SERVER_ADDRESS_STATIC,
// 		CONFIG_UDP_SAMPLE_SERVER_PORT);

// 	err = send(fd, buffer, sizeof(buffer), 0);
// 	if (err < 0) {
// 		LOG_ERR("Failed to transmit UDP packet, %d", -errno);
// 	}

// 	//  (void)k_work_reschedule(&server_transmission_work,
// 	//  			K_SECONDS(CONFIG_UDP_SAMPLE_DATA_UPLOAD_FREQUENCY_SECONDS));
// }

// static void video_preview_work_fn(struct k_work *work)
// {		LOG_INF("video_preview");
// 		video_preview();
// 		(void)k_work_reschedule(&video_preview_work,K_NO_WAIT);
// }

// // static int server_addr_construct(void)
// // {
// // 	int err;

// // 	struct sockaddr_in *server4 = ((struct sockaddr_in *)&client_addr);

// // 	server4->sin_family = AF_INET;
// // 	server4->sin_port = htons(CONFIG_UDP_SAMPLE_SERVER_PORT);
// // 	err = inet_pton(AF_INET, CONFIG_UDP_SAMPLE_SERVER_ADDRESS_STATIC, &server4->sin_addr);
// // 	if (err < 0) {
// // 		LOG_ERR("inet_pton, error: %d", -errno);
// // 		return err;
// // 	}
// // 	return 0;
// // }

// /*LED Blinking: Connecting to network*/
// /*LED ON: Connected to network*/
// void toggle_led(void)
// {
// 	int ret;

// 	if (!device_is_ready(led.port)) {
// 		LOG_ERR("LED device is not ready");
// 		return;
// 	}

// 	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
// 	if (ret < 0) {
// 		LOG_ERR("Error %d: failed to configure LED pin", ret);
// 		return;
// 	}

// 	while (1) {
// 		if (context.connected) {
//                         gpio_pin_set_dt(&led, 1);
// 			k_msleep(LED_SLEEP_TIME_MS);
// 		} else {
// 			gpio_pin_toggle_dt(&led);
// 			k_msleep(LED_SLEEP_TIME_MS);
// 		}
// 	}
// }

// K_THREAD_DEFINE(led_thread_id, 1024, toggle_led, NULL, NULL, NULL,
// 		7, 0, 0);

// static int cmd_wifi_status(void)
// {
// 	struct net_if *iface = net_if_get_default();
// 	struct wifi_iface_status status = { 0 };

// 	if (net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status,
// 				sizeof(struct wifi_iface_status))) {
// 		LOG_INF("Status request failed");

// 		return -ENOEXEC;
// 	}

// 	LOG_INF("==================");
// 	LOG_INF("WiFi State: %s", wifi_state_txt(status.state));

// 	if (status.state >= WIFI_STATE_ASSOCIATED) {
// 		uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];

// 		LOG_INF("Interface Mode: %s",
// 		       wifi_mode_txt(status.iface_mode));
// 		LOG_INF("Link Mode: %s",
// 		       wifi_link_mode_txt(status.link_mode));
// 		LOG_INF("SSID: %-32s", status.ssid);
// 		LOG_INF("BSSID: %s",
// 		       net_sprint_ll_addr_buf(
// 				status.bssid, WIFI_MAC_ADDR_LEN,
// 				mac_string_buf, sizeof(mac_string_buf)));
// 		LOG_INF("Band: %s", wifi_band_txt(status.band));
// 		LOG_INF("Channel: %d", status.channel);
// 		LOG_INF("Security: %s", wifi_security_txt(status.security));
// 		LOG_INF("MFP: %s", wifi_mfp_txt(status.mfp));
// 		LOG_INF("RSSI: %d", status.rssi);
// 	}
// 	return 0;
// }

// static void handle_wifi_connect_result(struct net_mgmt_event_callback *cb)
// {
// 	const struct wifi_status *status =
// 		(const struct wifi_status *) cb->info;

// 	if (context.connected) {
// 		return;
// 	}

// 	if (status->status) {
// 		LOG_ERR("Connection failed (%d)", status->status);
// 	} else {
// 		LOG_INF("Connected");
// 		context.connected = true;
// 	}

// 	context.connect_result = true;
// }

// static void handle_wifi_disconnect_result(struct net_mgmt_event_callback *cb)
// {
// 	const struct wifi_status *status =
// 		(const struct wifi_status *) cb->info;

// 	if (!context.connected) {
// 		return;
// 	}

// 	if (context.disconnect_requested) {
// 		LOG_INF("Disconnection request %s (%d)",
// 			 status->status ? "failed" : "done",
// 					status->status);
// 		context.disconnect_requested = false;
// 	} else {
// 		LOG_INF("Received Disconnected");
// 		context.connected = false;
// 	}

//         LOG_INF("Close UDP socket and stop transimtting");
//         (void)close(fd);
// 	(void)k_work_cancel_delayable(&server_transmission_work);

// }

// static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
// 				     uint32_t mgmt_event, struct net_if *iface)
// {
// 	switch (mgmt_event) {
// 	case NET_EVENT_WIFI_CONNECT_RESULT:
// 		handle_wifi_connect_result(cb);
// 		break;
// 	case NET_EVENT_WIFI_DISCONNECT_RESULT:
// 		handle_wifi_disconnect_result(cb);
// 		break;
// 	default:
// 		break;
// 	}
//         cmd_wifi_status();
// }

// static void on_net_event_dhcp_bound(struct net_mgmt_event_callback *cb)
// {
// 	/* Get DHCP info from struct net_if_dhcpv4 and print */
// 	const struct net_if_dhcpv4 *dhcpv4 = cb->info;
// 	const struct in_addr *addr = &dhcpv4->requested_ip;
// 	char dhcp_info[128];
//         //int err;


// 	net_addr_ntop(AF_INET, addr, dhcp_info, sizeof(dhcp_info));

// 	LOG_INF("WiFi Camera Server is ready on nRF7002DK, listening on %s:%d", dhcp_info,CONFIG_UDP_SAMPLE_SERVER_PORT);
// }
// static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
// 				    uint32_t mgmt_event, struct net_if *iface)
// {
// 	switch (mgmt_event) {
// 	case NET_EVENT_IPV4_DHCP_BOUND:
//                 on_net_event_dhcp_bound(cb);
//                 k_sem_give(&net_connected_sem);
//                 LOG_INF("given net_connected_sem");

// 		break;
// 	default:
// 		break;
// 	}
// }

// static int __wifi_args_to_params(struct wifi_connect_req_params *params)
// {

// 	params->timeout =  CONFIG_STA_CONN_TIMEOUT_SEC * MSEC_PER_SEC;

// 	if (params->timeout == 0) {
// 		params->timeout = SYS_FOREVER_MS;
// 	}

// 	/* SSID */
// 	params->ssid = CONFIG_STA_SAMPLE_SSID;
// 	params->ssid_length = strlen(params->ssid);

// #if defined(CONFIG_STA_KEY_MGMT_WPA2)
// 	params->security = 1;
// #elif defined(CONFIG_STA_KEY_MGMT_WPA2_256)
// 	params->security = 2;
// #elif defined(CONFIG_STA_KEY_MGMT_WPA3)
// 	params->security = 3;
// #else
// 	params->security = 0;
// #endif

// #if !defined(CONFIG_STA_KEY_MGMT_NONE)
// 	params->psk = CONFIG_STA_SAMPLE_PASSWORD;
// 	params->psk_length = strlen(params->psk);
// #endif
// 	params->channel = WIFI_CHANNEL_ANY;

// 	/* MFP (optional) */
// 	params->mfp = WIFI_MFP_OPTIONAL;

// 	return 0;
// }

// static int wifi_connect(void)
// {
// 	struct net_if *iface = net_if_get_default();
// 	static struct wifi_connect_req_params cnx_params;

// 	context.connected = false;
// 	context.connect_result = false;
// 	__wifi_args_to_params(&cnx_params);

// 	if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface,
// 		     &cnx_params, sizeof(struct wifi_connect_req_params))) {
// 		LOG_ERR("Connection request failed");

// 		return -ENOEXEC;
// 	}

// 	LOG_INF("Connection requested");

// 	return 0;
// }

// int bytes_from_str(const char *str, uint8_t *bytes, size_t bytes_len)
// {
// 	size_t i;
// 	char byte_str[3];

// 	if (strlen(str) != bytes_len * 2) {
// 		LOG_ERR("Invalid string length: %zu (expected: %d)\n",
// 			strlen(str), bytes_len * 2);
// 		return -EINVAL;
// 	}

// 	for (i = 0; i < bytes_len; i++) {
// 		memcpy(byte_str, str + i * 2, 2);
// 		byte_str[2] = '\0';
// 		bytes[i] = strtol(byte_str, NULL, 16);
// 	}

// 	return 0;
// }


int main(void)
{
// 	int ret;
// 	struct video_buffer *buffers[3];
// 	int i = 0;

// 	memset(&context, 0, sizeof(context));

// 	net_mgmt_init_event_callback(&wifi_shell_mgmt_cb,
// 				     wifi_mgmt_event_handler,
// 				     WIFI_SHELL_MGMT_EVENTS);

// 	net_mgmt_add_event_callback(&wifi_shell_mgmt_cb);


// 	net_mgmt_init_event_callback(&net_shell_mgmt_cb,
// 				     net_mgmt_event_handler,
// 				     NET_EVENT_IPV4_DHCP_BOUND);

// 	net_mgmt_add_event_callback(&net_shell_mgmt_cb);

//     // ret = server_addr_construct();
// 	// if (ret) {
// 	// 	LOG_INF("server_addr_construct, error: %d", ret);
// 	// 	FATAL_ERROR();
// 	// 	return ret;
// 	// }

// 	LOG_INF("Starting %s with CPU frequency: %d MHz", CONFIG_BOARD, SystemCoreClock/MHZ(1));
// 	k_sleep(K_SECONDS(1));

// #if defined(CONFIG_BOARD_NRF7002DK_NRF7001_NRF5340_CPUAPP) || \
// 	defined(CONFIG_BOARD_NRF7002DK_NRF5340_CPUAPP)
// 	if (strlen(CONFIG_NRF700X_QSPI_ENCRYPTION_KEY)) {
// 		char key[QSPI_KEY_LEN_BYTES];

// 		ret = bytes_from_str(CONFIG_NRF700X_QSPI_ENCRYPTION_KEY, key, sizeof(key));
// 		if (ret) {
// 			LOG_ERR("Failed to parse encryption key: %d\n", ret);
// 			return 0;
// 		}

// 		LOG_DBG("QSPI Encryption key: ");
// 		for (int i = 0; i < QSPI_KEY_LEN_BYTES; i++) {
// 			LOG_DBG("%02x", key[i]);
// 		}
// 		LOG_DBG("\n");

// 		ret = qspi_enable_encryption(key);
// 		if (ret) {
// 			LOG_ERR("Failed to enable encryption: %d\n", ret);
// 			return 0;
// 		}
// 		LOG_INF("QSPI Encryption enabled");
// 		} else {
// 			LOG_INF("QSPI Encryption disabled");
// 		}
// #endif /* CONFIG_BOARD_NRF700XDK_NRF5340 */

// 	    LOG_INF("Static IP address (overridable): %s/%s -> %s",
// 		CONFIG_NET_CONFIG_MY_IPV4_ADDR,
// 		CONFIG_NET_CONFIG_MY_IPV4_NETMASK,
// 		CONFIG_NET_CONFIG_MY_IPV4_GW);

//         wifi_connect();
//         while (!context.connect_result) {
//                 cmd_wifi_status();
//                 k_sleep(K_MSEC(STATUS_POLLING_MS));
//         }
		
// 	video = DEVICE_DT_GET(DT_NODELABEL(arducam_mega0));

// 	if (!device_is_ready(video)) {
// 		LOG_ERR("Video device %s not ready.", video->name);
// 		return -1;
// 	}

// 	/* Alloc video buffers and enqueue for capture */
// 	for (i = 0; i < ARRAY_SIZE(buffers); i++) {
// 		buffers[i] = video_buffer_alloc(1024);
// 		if (buffers[i] == NULL) {
// 			LOG_ERR("Unable to alloc video buffer");
// 			return -1;
// 		}
// 		video_enqueue(video, VIDEO_EP_OUT, buffers[i]);
// 	}

// 	LOG_INF("Mega start");

// 	printk("- Device name: %s\n", video->name);
// 	video_stream_stop(video);
// 	video_set_ctrl(video, VIDEO_CID_ARDUCAM_RESET, NULL);

// 	struct sockaddr_in client_address;
// 	//socklen_t client_address_len = sizeof(client_address);
        
//         struct sockaddr_in server_address;
// 	//socklen_t server_address_len = sizeof(server_address);
// 	char addr_str[32];

//         k_msgq_get(&addr_queue, &client_address, K_FOREVER);

//         inet_ntop(client_address.sin_family, &client_address.sin_addr,addr_str, sizeof(addr_str));
//         LOG_INF("client IPAddr = %s, Port = %d\n", addr_str, ntohs(client_address.sin_port));
        
//         fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
// 	if (fd < 0) {
// 		LOG_ERR("Failed to create UDP socket: %d", -errno);
// 		FATAL_ERROR();
// 		return 0;
// 	}
	
// 	server_address.sin_family = AF_INET;
// 	server_address.s
// in_addr.s_addr = htonl(INADDR_ANY);
// 	server_address.sin_port = htons(50005);
//         ret =  bind(fd, (struct sockaddr *)&server_address, sizeof(server_address));
//         if (ret < 0) {
// 		LOG_ERR("bind, error: %d", -errno);
// 		FATAL_ERROR();
// 		return 0;
// 	}

//         client_address.sin_port = htons(60006);
//         ret =  connect(fd, (struct sockaddr *)&client_address, sizeof(struct sockaddr_in));
//         if (ret < 0) {
//                 LOG_ERR("connect, error: %d", -errno);
//                 FATAL_ERROR();
//                 return 0;
//         }

//         LOG_INF("Before while");
// 	while (1) {
//                 ret = k_msgq_get(&udp_recv_queue, &udp_recive_buf, K_MSEC(1));
//                 if(ret == 0) {
//                         LOG_INF("k_msgq_get");
//                         ret = process_udp_rx_buffer(udp_recive_buf, command_buf);
//                         if (ret > 0) {
//                                 LOG_INF("Valid command reviced. Length:%d Value:",ret);
//                                 for (uint8_t i = 0; i < ret; i++) {
//                                         LOG_INF("%02X ", command_buf[i] & 0xFF);
//                                 }
//                                 recv_process(command_buf);
//                         }else{
//                                 LOG_INF("No valid command reviced");
//                         }
//                 }
//                 if(preview_on==1){
//                         video_preview();
//                         //k_msleep(1);
//                 }
//                 k_yield();
// 	}
	return 0;
}


// /*
// ** Two sockets are use:
// ** Client(host):6000 -> Server(nRF7002DK):5000 
// ** Client(host):6006 -> Server(nRF7002DK):5005 
// **
// */

// void server_udp_recv(void){
//         int ret;
//         static int fd_host;
//         struct sockaddr_in client_addr;
//         socklen_t client_addr_len;
//         char addr_str[32];
        
//         k_sem_take(&net_connected_sem, K_FOREVER);
//         LOG_INF("taken net_connected_sem");

//         fd_host = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
// 	if (fd_host < 0) {
// 		LOG_ERR("Failed to create UDP socket: %d", -errno);
// 		FATAL_ERROR();
// 		return;
// 	}
        
//         client_addr.sin_family = AF_INET;
// 	client_addr.sin_addr.s_addr = htonl(INADDR_ANY);
// 	client_addr.sin_port = htons(50000);
	
//         ret =  bind(fd_host, (struct sockaddr *)&client_addr, sizeof(client_addr));
//         if (ret < 0) {
// 		LOG_ERR("bind, error: %d", -errno);
// 		FATAL_ERROR();
// 		return;
// 	}

//         ret = recvfrom(fd_host, udp_recv_buf, sizeof(udp_recv_buf),0, (struct sockaddr* )&client_addr, &client_addr_len);
// 	if (ret < 0) {
// 		LOG_ERR("recvfrom, error: %d", -errno);
// 		FATAL_ERROR();
// 		return;
// 	}

//         inet_ntop(client_addr.sin_family, &client_addr.sin_addr,addr_str, sizeof(addr_str));
//         LOG_INF("Host IPAddr = %s, Port = %d\n", addr_str, ntohs(client_addr.sin_port));
//         k_msgq_put(&addr_queue, &client_addr, K_NO_WAIT);

//         LOG_INF("udp_rx_buf content:");
//         for (size_t i = 0; i < COMMAND_MAX_SIZE; ++i) {
//                 printk("%02X ", udp_recv_buf[i]);
//         }
//         printk("\n");
        
//         while(1){
                
//                 ret = recvfrom(fd_host, udp_recv_buf, sizeof(udp_recv_buf),0, (struct sockaddr* )&client_addr, &client_addr_len);
//                 LOG_INF("recived udp_rx_buf");
//                 if (ret < 0) {
//                         LOG_ERR("[%d] Cannot receive udp message (%d)", fd,
//                                 -errno);
//                         return;
//                 }
//                 k_msgq_put(&udp_recv_queue, &udp_recv_buf, K_NO_WAIT);
//                 k_yield();
//         }
// }

// /* size of stack area used by each thread */
// #define STACKSIZE 1024
// /* scheduling priority used by each thread */
// #define PRIORITY 7

// K_THREAD_DEFINE(server_udp_recv_id, STACKSIZE, server_udp_recv, NULL, NULL, NULL,
// 		PRIORITY, 0, 0);
