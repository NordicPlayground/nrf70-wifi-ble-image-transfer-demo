/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 * @brief WiFi Camera Demo
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(WiFiCam, CONFIG_LOG_DEFAULT_LEVEL);

#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M
#include <nrfx_clock.h>
#endif

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/video.h>
#include <zephyr/drivers/video/arducam_mega.h>
#include <zephyr/net/socket.h>

#include "app_bluetooth.h"
#include "net_util.h"

extern int socket_send;
extern uint8_t udp_recv_buf[];
extern struct k_msgq udp_recv_queue;

#define RESET_CAMERA 0XFF
#define SET_PICTURE_RESOLUTION 0X01
#define SET_VIDEO_RESOLUTION 0X02
#define SET_BRIGHTNESS 0X03
#define SET_CONTRAST 0X04
#define SET_SATURATION 0X05
#define SET_EV 0X06
#define SET_WHITEBALANCE 0X07
#define SET_SPECIAL_EFFECTS 0X08
#define SET_FOCUS_ENABLE 0X09
#define SET_EXPOSURE_GAIN_ENABLE 0X0A
#define SET_WHITE_BALANCE_ENABLE 0X0C
#define SET_MANUAL_GAIN 0X0D
#define SET_MANUAL_EXPOSURE 0X0E
#define GET_CAMERA_INFO 0X0F
#define TAKE_PICTURE 0X10
#define SET_SHARPNESS 0X11
#define DEBUG_WRITE_REGISTER 0X12
#define STOP_STREAM 0X21
#define GET_FRM_VER_INFO 0X30
#define GET_SDK_VER_INFO 0X40
#define SET_IMAGE_QUALITY 0X50
#define SET_LOWPOWER_MODE 0X60

/*
** Arducam mega communication protocols
** https://www.arducam.com/docs/arducam-mega/arducam-mega-getting-started/packs/HostCommunicationProtocol.html
*/

const struct device *video;
struct video_buffer *vbuf;
static struct arducam_mega_info mega_info;

enum client_type {
	client_type_NONE,
	client_type_BLE,
	client_type_UDP
};

enum app_command_type {APPCMD_CAM_COMMAND, APPCMD_TAKE_PICTURE, APPCMD_START_STOP_STREAM, APPCMD_UDP_RX};

struct app_command_t {
	enum app_command_type type;
	enum client_type mode;
	uint8_t cam_cmd;
	uint8_t data[6];
};

K_MSGQ_DEFINE(msgq_app_commands, sizeof(struct app_command_t), 8, 4);

struct client_state {
	uint8_t req_stream : 1;
	uint8_t req_stream_stop : 1;
	uint8_t stream_active : 1;
};
struct client_state client_state_ble = {0};
struct client_state client_state_udp = {0};

static void on_timer_count_bytes_func(struct k_timer *timer);
K_TIMER_DEFINE(m_timer_count_bytes, on_timer_count_bytes_func, NULL);
static int counted_bytes_sent = 0;

const uint32_t pixel_format_table[] = {
	VIDEO_PIX_FMT_JPEG,
	VIDEO_PIX_FMT_RGB565,
	VIDEO_PIX_FMT_YUYV,
};

const uint16_t resolution_table[][2] = {
	{160, 120},
	{320, 240},
	{640, 480},
	{800, 600},
	{1280, 720},
	{1280, 960},
	{1600, 1200},
	{1920, 1080},
	{2048, 1536},
	{2592, 1944},
	{96, 96},
	{128, 128},
	{320, 320},
};

const uint8_t resolution_num = sizeof(resolution_table) / 4;

static uint8_t current_resolution;
static uint8_t target_resolution;
static uint8_t take_picture_fmt = 0x1a;

void cam_to_host_command_send(uint8_t type, uint8_t *buffer, uint32_t length)
{
	udp_head_and_tail[2] = type;
	send(socket_send, &udp_head_and_tail[0], 3, 0);
	if(length!=0){
		send(socket_send, (uint8_t *)&length, 4, 0);
		send(socket_send, buffer, length, 0);
	}
	send(socket_send, &udp_head_and_tail[3], 2, 0);
}

int set_mega_resolution(uint8_t sfmt, enum client_type source_client)
{
	uint8_t resolution = sfmt & 0x0f;
	uint8_t pixelformat = (sfmt & 0x70) >> 4;

	// If the source is the Bluetooth client, and one of the max resolutions are selected (3MP or 5MP), ensure the right max resolution is selected based on the connected camera type
	if (source_client == client_type_BLE && (resolution == 8 || resolution == 9)) {
		resolution = ((mega_info.camera_id == ARDUCAM_SENSOR_3MP_1 || mega_info.camera_id == ARDUCAM_SENSOR_3MP_2)) ? 8 : 9;
	}

	if (resolution > resolution_num || pixelformat > 3)
	{
		return -1;
	}
	struct video_format fmt = {.width = resolution_table[resolution][0],
							   .height = resolution_table[resolution][1],
							   .pixelformat = pixel_format_table[pixelformat - 1]};
	// If the resolution has changed, and the client that initiated the change was UDP, notify the BLE client
	if (resolution != current_resolution) {
		if (source_client == client_type_UDP) {
			app_bt_send_client_status(((mega_info.camera_id == ARDUCAM_SENSOR_3MP_1 || mega_info.camera_id == ARDUCAM_SENSOR_3MP_2)) ? 1 : 2, resolution);
		} else if (source_client == client_type_BLE) {
			cam_to_host_command_send(0x07, &resolution, sizeof(resolution));
		}
	}

	current_resolution = resolution;

	return video_set_format(video, VIDEO_EP_OUT, &fmt);
}

void send_picture_data_udp(uint8_t *data, int length)
{
	counted_bytes_sent += length;
	while (length >= 1024) {
		send(socket_send, data, 1024, 0); 
		data += 1024;
		length -= 1024;
	}
	if (length > 0) {
		send(socket_send, data, length, 0); 
	}
}

void send_picture_data_ble(uint8_t *data, int length)
{
	counted_bytes_sent += length;
	app_bt_send_picture_data(data, length);
}

static void client_request_single_capture(struct client_state *state)
{
	if (!state->stream_active) {
		state->req_stream = 1;
		state->req_stream_stop = 1;
	}
}

static void client_request_stream_start_stop(struct client_state *state, bool start)
{
	if (start) {
		state->req_stream = 1;
	} else {
		state->req_stream_stop = 1;
	}
}

static int client_check_start_request(struct client_state *state)
{
	if (state->req_stream && !state->stream_active) {
		state->req_stream = 0;
		state->stream_active = 1;
		return 1;
	}
	return 0;
}

static int client_check_stop_request(struct client_state *state)
{
	if (state->req_stream_stop && state->stream_active) {
		state->req_stream_stop = 0;
		state->stream_active = 0;
		return 1;
	}
	return 0;
}

void video_preview(void)
{
	int err;
	static bool capture_flag = false;
	enum video_frame_fragmented_status f_status;

	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
	if (err)
	{
		LOG_INF("Unable to dequeue video buf");
		return;
	}
	f_status = vbuf->flags;
	
	LOG_DBG("CF1: f_status %i, bytesframe %i, bytesused %i. cf %i", f_status, vbuf->bytesframe, vbuf->bytesused, capture_flag);
	
	if (capture_flag)
	{
		capture_flag = false;

		if(f_status == VIDEO_BUF_EOF) {
			LOG_WRN("Capture flag set while status EOF. Skipping!");
			err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
			if (err)
			{
				LOG_ERR("Unable to requeue video buf");
				return;
			}
			return;
		}

		client_check_start_request(&client_state_ble);
		client_check_start_request(&client_state_udp);

		if (client_state_ble.stream_active) {
			app_bt_send_picture_header(vbuf->bytesframe);
		} 
		if (client_state_udp.stream_active) {
			udp_head_and_tail[2] = 0x01;
			send(socket_send, &udp_head_and_tail[0], 3, 0);
			send(socket_send, (uint8_t *)&vbuf->bytesframe, 4, 0);
			target_resolution = (((current_resolution & 0x0f) << 4) | 0x01);
			send(socket_send, &target_resolution, 1, 0);
		}
	}

	if (client_state_ble.stream_active) {
		send_picture_data_ble(vbuf->buffer, vbuf->bytesused);
	} 
	if (client_state_udp.stream_active) {
		send_picture_data_udp(vbuf->buffer, vbuf->bytesused);
	}

	if (f_status == VIDEO_BUF_EOF)
	{
		capture_flag = true;

		if (client_state_udp.stream_active) {
			send(socket_send, &udp_head_and_tail[3], 2, 0);
		}

		client_check_stop_request(&client_state_ble);
		client_check_stop_request(&client_state_udp);	
	}

	err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
	if (err)
	{
		LOG_ERR("Unable to requeue video buf");
		return;
	}
}

int report_mega_info(void)
{
	static char str_buf[400];
	uint32_t str_len;
	char *mega_type;

	switch (mega_info.camera_id)
	{
	case ARDUCAM_SENSOR_3MP_1:
	case ARDUCAM_SENSOR_3MP_2:
		mega_type = "3MP";
		break;
	case ARDUCAM_SENSOR_5MP_1:
		mega_type = "5MP";
		break;
	case ARDUCAM_SENSOR_5MP_2:
		mega_type = "5MP_2";
		break;
	default:
		return -ENODEV;
	}

	sprintf(str_buf,
			"ReportCameraInfo\r\nCamera Type:%s\r\n"
			"Camera Support Resolution:%d\r\nCamera Support "
			"special effects:%d\r\nCamera Support Focus:%d\r\n"
			"Camera Exposure Value Max:%ld\r\nCamera Exposure Value "
			"Min:%d\r\nCamera Gain Value Max:%d\r\nCamera Gain Value "
			"Min:%d\r\nCamera Support Sharpness:%d\r\n",
			mega_type, mega_info.support_resolution, mega_info.support_special_effects,
			mega_info.enable_focus, mega_info.exposure_value_max, mega_info.exposure_value_min,
			mega_info.gain_value_max, mega_info.gain_value_min, mega_info.enable_sharpness);
	printk("%s", str_buf);
	str_len = strlen(str_buf);
	cam_to_host_command_send(0x02, str_buf, str_len);
	return 0;
}

uint8_t recv_process(uint8_t *buff)
{
	LOG_INF("recv_process: cmd %x, data %x", buff[0], buff[1]);
	switch (buff[0])
	{
	case SET_PICTURE_RESOLUTION:
		LOG_INF("camcmd: SET_PICTURE_RESOLUTION");
		if (set_mega_resolution(buff[1], client_type_UDP) == 0)
		{
			take_picture_fmt = buff[1];
		}
		break;
	case SET_VIDEO_RESOLUTION:
		LOG_INF("camcmd: SET_VIDEO_RESOLUTION");
		if (!client_state_udp.stream_active)
		{
			set_mega_resolution(buff[1] | 0x10, client_type_UDP);
			client_request_stream_start_stop(&client_state_udp, true);
		}
		break;
	case SET_BRIGHTNESS:
		video_set_ctrl(video, VIDEO_CID_CAMERA_BRIGHTNESS, &buff[1]);
		break;
	case SET_CONTRAST:
		video_set_ctrl(video, VIDEO_CID_CAMERA_CONTRAST, &buff[1]);
		break;
	case SET_SATURATION:
		video_set_ctrl(video, VIDEO_CID_CAMERA_SATURATION, &buff[1]);
		break;
	case SET_EV:
		video_set_ctrl(video, VIDEO_CID_ARDUCAM_EV, &buff[1]);
		break;
	case SET_WHITEBALANCE:
		video_set_ctrl(video, VIDEO_CID_CAMERA_WHITE_BAL, &buff[1]);
		break;
	case SET_SPECIAL_EFFECTS:
		video_set_ctrl(video, VIDEO_CID_ARDUCAM_COLOR_FX, &buff[1]);
		break;
	case SET_EXPOSURE_GAIN_ENABLE:
		video_set_ctrl(video, VIDEO_CID_CAMERA_EXPOSURE_AUTO, &buff[1]);
		video_set_ctrl(video, VIDEO_CID_CAMERA_GAIN_AUTO, &buff[1]);
	case SET_WHITE_BALANCE_ENABLE:
		video_set_ctrl(video, VIDEO_CID_CAMERA_WHITE_BAL_AUTO, &buff[1]);
		break;
	case SET_SHARPNESS:
		video_set_ctrl(video, VIDEO_CID_ARDUCAM_SHARPNESS, &buff[1]);
		break;
	case SET_MANUAL_GAIN:
		uint16_t gain_value = (buff[1] << 8) | buff[2];

		video_set_ctrl(video, VIDEO_CID_CAMERA_GAIN, &gain_value);
		break;
	case SET_MANUAL_EXPOSURE:
		uint32_t exposure_value = (buff[1] << 16) | (buff[2] << 8) | buff[3];

		video_set_ctrl(video, VIDEO_CID_CAMERA_EXPOSURE, &exposure_value);
		break;
	case GET_CAMERA_INFO:
		report_mega_info();
		break;
	case TAKE_PICTURE:
		LOG_INF("Take picture");
		client_request_single_capture(&client_state_udp);
		break;
	case STOP_STREAM:
		if (client_state_udp.stream_active)
		{
			LOG_INF("Stop video stream");
			client_request_stream_start_stop(&client_state_udp, false);
		}
		break;
	case RESET_CAMERA:
		video_set_ctrl(video, VIDEO_CID_ARDUCAM_RESET, NULL);
		break;
	case SET_IMAGE_QUALITY:
		video_set_ctrl(video, VIDEO_CID_JPEG_COMPRESSION_QUALITY, &buff[1]);
		break;
	case SET_LOWPOWER_MODE:
		video_set_ctrl(video, VIDEO_CID_ARDUCAM_LOWPOWER, &buff[1]);
		break;
	default:
		break;
	}

	return buff[0];
}

static void register_app_command(const struct app_command_t *command)
{
	if (k_msgq_put(&msgq_app_commands, command, K_NO_WAIT) != 0) {
		LOG_ERR("Command buffer full!");
	}
}

void app_bt_connected_callback(void)
{	
	LOG_INF("Bluetooth connection established");
	//send command 0x08 to inform WiFi Host BLE client is connected.
	cam_to_host_command_send(0x08, NULL,0);
}

void app_bt_ready_callback(void)
{
	LOG_INF("Bluetooth client ready");
	app_bt_send_client_status(((mega_info.camera_id == ARDUCAM_SENSOR_3MP_1 || mega_info.camera_id == ARDUCAM_SENSOR_3MP_2)) ? 1 : 2, current_resolution);
	
}

void app_bt_disconnected_callback(void)
{
	LOG_INF("Bluetooth disconnected");
	
	client_state_ble.req_stream = 0;
	client_state_ble.req_stream_stop = 0;
	client_state_ble.stream_active = 0;
	//send command 0x09 to inform WiFi Host BLE client is disconnected.
	cam_to_host_command_send(0x09, NULL,0);
}

void app_bt_take_picture_callback(void)
{
	static struct app_command_t cmd_take_picture = 
		{.type = APPCMD_TAKE_PICTURE, .mode = client_type_BLE,};
	register_app_command(&cmd_take_picture);
}

void app_bt_enable_stream_callback(bool enable)
{
	static struct app_command_t cmd_stream_start_stop = 
		{.type = APPCMD_START_STOP_STREAM, .mode = client_type_BLE,};
	cmd_stream_start_stop.data[0] = enable ? 1 : 0;
	register_app_command(&cmd_stream_start_stop);
}

void app_bt_change_resolution_callback(uint8_t resolution)
{
	static struct app_command_t cmd_take_picture = 
		{.type = APPCMD_CAM_COMMAND, 
		 .cam_cmd = SET_PICTURE_RESOLUTION};
	cmd_take_picture.data[0] = 0x10 | (resolution & 0xF);
	register_app_command(&cmd_take_picture);

	//current_resolution = resolution;
}

const struct app_bt_cb app_bt_callbacks = {
	.connected = app_bt_connected_callback,
	.ready = app_bt_ready_callback,
	.disconnected = app_bt_disconnected_callback,
    .take_picture = app_bt_take_picture_callback,
	.enable_stream = app_bt_enable_stream_callback,
	.change_resolution = app_bt_change_resolution_callback,
};

static void on_timer_count_bytes_func(struct k_timer *timer)
{
	if (counted_bytes_sent > 0) {
		LOG_INF("Data transferred: %i kbps", (counted_bytes_sent * 8) / 1024);
		counted_bytes_sent = 0;
	}
}

void udp_rx_callback(uint8_t *data, uint16_t len)
{
	static struct app_command_t app_cmd_udp = {.type = APPCMD_UDP_RX};
	LOG_DBG("UDP RX callback");
	memcpy(app_cmd_udp.data, data, len);
	register_app_command(&app_cmd_udp);
}

int main(void)
{
	int ret;
	struct video_buffer *buffers[3];
#if defined(CLOCK_FEATURE_HFCLK_DIVIDE_PRESENT) || NRF_CLOCK_HAS_HFCLK192M

	/* hardcode to 128MHz */
	nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);
#endif
	k_sleep(K_SECONDS(1));
	printk("Starting %s with CPU frequency: %d MHz\n", CONFIG_BOARD, SystemCoreClock / MHZ(1));

	video = DEVICE_DT_GET(DT_NODELABEL(arducam_mega0));

	if (!device_is_ready(video))
	{
		LOG_ERR("Video device %s not ready.", video->name);
		return -1;
	}

	video_stream_stop(video);
	LOG_INF("Device %s is ready!", video->name);

	video_get_ctrl(video, VIDEO_CID_ARDUCAM_INFO, &mega_info);

	ret = app_bt_init(&app_bt_callbacks);
	if (ret < 0) {
		LOG_ERR("Error initializing Bluetooth");
		return -1;
	}

	net_util_set_callback(udp_rx_callback);

	/* Alloc video buffers and enqueue for capture */
	for (int i = 0; i < ARRAY_SIZE(buffers); i++)
	{
		buffers[i] = video_buffer_alloc(4096);
		if (buffers[i] == NULL)
		{
			LOG_ERR("Unable to alloc video buffer");
			return -1;
		}
		video_enqueue(video, VIDEO_EP_OUT, buffers[i]);
	}

	k_timer_start(&m_timer_count_bytes, K_MSEC(1000), K_MSEC(1000));
	
	set_mega_resolution(0x11, client_type_NONE);

	video_stream_start(video);
	
	while (1) {
		struct app_command_t new_command;
		if (k_msgq_get(&msgq_app_commands, &new_command, K_USEC(50)) == 0) {
			switch (new_command.type) {
				case APPCMD_TAKE_PICTURE:
					LOG_INF("TAKE PICTURE");
					client_request_single_capture(&client_state_ble);
					break;
				case APPCMD_CAM_COMMAND:
					switch (new_command.cam_cmd) {
						case SET_PICTURE_RESOLUTION:
							LOG_INF("Change resolution to 0x%x", new_command.data[0]);
							set_mega_resolution(new_command.data[0], client_type_BLE);
							break;
					}
					break;
				case APPCMD_START_STOP_STREAM:
					bool enable = new_command.data[0] > 0; 
					
					if (enable) {
						LOG_INF("Starting stream!");
						client_request_stream_start_stop(&client_state_ble, true);
					} else {
						LOG_INF("Stopping stream");
						client_request_stream_start_stop(&client_state_ble, false);
					}
					break;
				case APPCMD_UDP_RX:
					uint8_t udp_cmd_buf[UDP_COMMAND_MAX_SIZE - 2];
					ret = process_udp_rx_buffer(new_command.data, udp_cmd_buf);
					if (ret > 0)
					{
						LOG_INF("Valid command received. Length:%d", ret);
						LOG_HEXDUMP_INF(udp_cmd_buf, ret, "Data: ");
						recv_process(udp_cmd_buf);
					}
					else
					{
						LOG_INF("Invalid UDP command received");
					}
					break;
			}
		}

		video_preview();
	}
	return 0;
}