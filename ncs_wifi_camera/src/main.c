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

extern int socket_send;
extern uint8_t udp_recv_buf[];
extern struct k_msgq udp_recv_queue;
extern bool is_socket_ready;

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
#define COMMAND_MAX_SIZE 6
/* actucal command not count start(0x55) and stop(0xAA) codes*/
static uint8_t command_buf[COMMAND_MAX_SIZE - 2];
static uint8_t head_and_tail[] = {0xff, 0xaa, 0x00, 0xff, 0xbb};

const struct device *video;
struct video_buffer *vbuf;

static bool preview_on = false;
static bool capture_flag = false;
static bool request_stream_stop = false;

static enum APP_MODE {
	APP_MODE_BLE,
	APP_MODE_UDP
} app_mode_current = APP_MODE_UDP;

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
	head_and_tail[2] = type;
	send(socket_send, &head_and_tail[0], 3, 0);
	send(socket_send, (uint8_t *)&length, 4, 0);
	send(socket_send, buffer, length, 0);
	send(socket_send, &head_and_tail[3], 2, 0);
}

int set_mega_resolution(uint8_t sfmt)
{
	uint8_t resolution = sfmt & 0x0f;
	uint8_t pixelformat = (sfmt & 0x70) >> 4;

	if (resolution > resolution_num || pixelformat > 3)
	{
		return -1;
	}
	struct video_format fmt = {.width = resolution_table[resolution][0],
							   .height = resolution_table[resolution][1],
							   .pixelformat = pixel_format_table[pixelformat - 1]};
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

int take_picture(enum APP_MODE mode)
{
	int err;
	enum video_frame_fragmented_status f_status;

	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
	if (err)
	{
		LOG_ERR("Unable to dequeue video buf");
		return -1;
	}

	f_status = vbuf->flags;

	if (mode == APP_MODE_UDP) {
		head_and_tail[2] = 0x01;
		send(socket_send, &head_and_tail[0], 3, 0);
		send(socket_send, (uint8_t *)&vbuf->bytesframe, 4, 0); 
		target_resolution = (((current_resolution & 0x0f) << 4) | 0x01);
		send(socket_send, &target_resolution, 1, 0);
		send_picture_data_udp(vbuf->buffer, vbuf->bytesused);
	} else {
		app_bt_send_picture_header(vbuf->bytesframe);
		send_picture_data_ble(vbuf->buffer, vbuf->bytesused);
	}

	video_enqueue(video, VIDEO_EP_OUT, vbuf);
	while (f_status == VIDEO_BUF_FRAG)
	{
		video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
		f_status = vbuf->flags;
		if (mode == APP_MODE_UDP) {
			send_picture_data_udp(vbuf->buffer, vbuf->bytesused);
		} else {
			send_picture_data_ble(vbuf->buffer, vbuf->bytesused);
		}
		video_enqueue(video, VIDEO_EP_OUT, vbuf);
	}

	if (mode == APP_MODE_UDP) {
		send(socket_send, &head_and_tail[3], 2, 0);
	}

	return 0;
}

void video_preview(enum APP_MODE mode)
{
	int err;
	enum video_frame_fragmented_status f_status;
	if (!preview_on)
	{
		return;
	}
	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
	if (err)
	{
		LOG_INF("Unable to dequeue video buf");
		return;
	}
	f_status = vbuf->flags;

	if (capture_flag)
	{
		capture_flag = false;

		if (mode == APP_MODE_BLE) {
			app_bt_send_picture_header(vbuf->bytesframe);
		} else {
			head_and_tail[2] = 0x01;
			send(socket_send, &head_and_tail[0], 3, 0);
			send(socket_send, (uint8_t *)&vbuf->bytesframe, 4, 0);
			target_resolution = (((current_resolution & 0x0f) << 4) | 0x01);
			send(socket_send, &target_resolution, 1, 0);
		}
	}

	if (mode == APP_MODE_BLE) {
		send_picture_data_ble(vbuf->buffer, vbuf->bytesused);
	} else {
		send_picture_data_udp(vbuf->buffer, vbuf->bytesused);
	}

	if (f_status == VIDEO_BUF_EOF)
	{
		if (mode == APP_MODE_UDP) {
			send(socket_send, &head_and_tail[3], 2, 0);
		}
		capture_flag = true;

		if(request_stream_stop) {
			request_stream_stop = false;
			video_stream_stop(video);
			preview_on = false;
		}
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
	struct arducam_mega_info mega_info;

	video_get_ctrl(video, VIDEO_CID_ARDUCAM_INFO, &mega_info);

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
	while(is_socket_ready == false);
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

		if (set_mega_resolution(buff[1]) == 0)
		{
			take_picture_fmt = buff[1];
		}
		break;
	case SET_VIDEO_RESOLUTION:
		LOG_INF("camcmd: SET_VIDEO_RESOLUTION");
		if (!preview_on)
		{
			LOG_INF("SET_VIDEO_RESOLUTION preview_on");
			set_mega_resolution(buff[1] | 0x10);
			video_stream_start(video);
			LOG_INF("start video stream");
			capture_flag = true;
		}
		preview_on = true;
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
		video_stream_start(video);
		take_picture(APP_MODE_UDP);
		video_stream_stop(video);
		LOG_INF("take picture");
		break;
	case STOP_STREAM:
		if (preview_on)
		{
			send(socket_send, &head_and_tail[3], 2, 0);
			video_stream_stop(video);
			set_mega_resolution(take_picture_fmt);
			LOG_INF("start video stream");
		}
		preview_on = false;
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

uint8_t process_udp_rx_buffer(char *udp_rx_buf, char *command_buf)
{
	uint8_t command_length = 0;

	// Check if udp_rx_buf contains start code (0x55)
	if (udp_rx_buf[0] == 0x55)
	{
		// Find the end code (0xAA) and extract the command
		for (uint8_t i = 1; i < COMMAND_MAX_SIZE; i++)
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

void app_bt_connected_callback(void)
{	
	LOG_INF("Bluetooth connection established. Entering BLE app mode");

	// If we are in streaming mode, stop the streaming
	if (preview_on) {
		video_stream_stop(video);
		preview_on = false;
	}

	// Set the default resolution for the BT app
	set_mega_resolution(0x11);

	app_mode_current = APP_MODE_BLE;
}

void app_bt_disconnected_callback(void)
{
	LOG_INF("Bluetooth disconnected. Entering UDP app mode");
	
	// If we are in streaming mode, stop the streaming
	if (preview_on) {
		video_stream_stop(video);
		preview_on = false;
	}

	app_mode_current = APP_MODE_UDP;
}

void app_bt_take_picture_callback(void)
{
	LOG_INF("TAKE PICTURE");
	video_stream_start(video);
	take_picture(APP_MODE_BLE);
	video_stream_stop(video);
}

void app_bt_enable_stream_callback(bool enable)
{
	// If we are already in the same state, exit
	if(enable == preview_on) return;
	
	if (enable) {
		LOG_INF("Starting stream!");
		video_stream_start(video);
		capture_flag = true;
		preview_on = true;
	} else {
		LOG_INF("Stopping stream");
		request_stream_stop = true;
	}
}

void app_bt_change_resolution_callback(uint8_t resolution)
{
	LOG_INF("Change resolution to %i", resolution);
	set_mega_resolution(0x10 | (resolution & 0xF));
}

const struct app_bt_cb app_bt_callbacks = {
	.connected = app_bt_connected_callback,
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

	ret = app_bt_init(&app_bt_callbacks);
	if (ret < 0) {
		LOG_ERR("Error initializing Bluetooth");
		return -1;
	}

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

	//k_sem_take(&sockets_ready, K_FOREVER);

	k_timer_start(&m_timer_count_bytes, K_MSEC(1000), K_MSEC(1000));

	while (1)
	{
		if (app_mode_current == APP_MODE_UDP) {
			ret = k_msgq_get(&udp_recv_queue, &udp_recv_buf, K_USEC(10));
			if (ret == 0)
			{
				ret = process_udp_rx_buffer(udp_recv_buf, command_buf);
				if (ret > 0)
				{
					LOG_INF("Valid command received. Length:%d", ret);
					LOG_HEXDUMP_INF(command_buf, ret, "Data: ");
					recv_process(command_buf);
				}
				else
				{
					LOG_INF("No valid command reviced");
				}
			}
			if (preview_on == 1)
			{
				video_preview(APP_MODE_UDP);
			}			
			k_yield();
		} else if (app_mode_current == APP_MODE_BLE) {
			if (preview_on == 1)
			{
				video_preview(APP_MODE_BLE);
			} else {
				k_msleep(100);
			}
		}
	}
	return 0;
}