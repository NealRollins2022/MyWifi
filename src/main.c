/**
 * Copyright (c) 2023 Arducam Technology Co., Ltd. <www.arducam.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <dk_buttons_and_leds.h>
#include <drivers/video.h>
#include <drivers/video/arducam_mega.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <net/wifi_mgmt_ext.h>
#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main);
#define EVENT_MASK (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)
#define RESET_CAMERA             0XFF
#define SET_PICTURE_RESOLUTION   0X01
#define SET_VIDEO_RESOLUTION     0X02
#define SET_BRIGHTNESS           0X03
#define SET_CONTRAST             0X04
#define SET_SATURATION           0X05
#define SET_EV                   0X06
#define SET_WHITEBALANCE         0X07
#define SET_SPECIAL_EFFECTS      0X08
#define SET_FOCUS_ENABLE         0X09
#define SET_EXPOSURE_GAIN_ENABLE 0X0A
#define SET_WHITE_BALANCE_ENABLE 0X0C
#define SET_MANUAL_GAIN          0X0D
#define SET_MANUAL_EXPOSURE      0X0E
#define GET_CAMERA_INFO          0X0F
#define TAKE_PICTURE             0X10
#define SET_SHARPNESS            0X11
#define DEBUG_WRITE_REGISTER     0X12
#define STOP_STREAM              0X21
#define GET_FRM_VER_INFO         0X30
#define GET_SDK_VER_INFO         0X40
#define SET_IMAGE_QUALITY        0X50
#define SET_LOWPOWER_MODE        0X60

#define MSG_SIZE 12

#define NUM_BUFFERS 3
#define MAX_SPI_BURST 4096
#define DMA_BUF_SIZE  (MAX_SPI_BURST + 8)
static struct net_mgmt_event_callback mgmt_cb;
static bool connected;
static K_SEM_DEFINE(run_app, 0, 1);
/* DMA-safe buffers for video frames */
static uint8_t dma_bufs[NUM_BUFFERS][DMA_BUF_SIZE] __aligned(4)
                         __attribute__((section(".dma")));

/* Video buffer structs */
struct video_buffer video_buffers[NUM_BUFFERS];
/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

const struct device *console;
const struct device *video;
struct video_buffer *vbuf;

volatile uint8_t preview_on;
volatile uint8_t capture_flag;
static void net_mgmt_event_handler(struct net_mgmt_event_callback *cb,
                                   uint32_t mgmt_event, struct net_if *iface)
{
    if ((mgmt_event & EVENT_MASK) != mgmt_event) {
        return;
    }

    if (mgmt_event == NET_EVENT_L4_CONNECTED) {
        LOG_INF("Wi-Fi connected");
        connected = true;
        dk_set_led_on(DK_LED1);
        k_sem_give(&run_app);  // unblock main thread -> camera can start now
    } else if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
        connected = false;
        dk_set_led_off(DK_LED1);
        LOG_INF("Wi-Fi disconnected");
    }
}

/* STEP 7 - Define the function to populate the Wi-Fi credential parameters */
static int wifi_args_to_params(struct wifi_connect_req_params *params)
{

	/* STEP 7.1 Populate the SSID and password */
	params->ssid = CONFIG_WIFI_CREDENTIALS_STATIC_SSID;
	params->ssid_length = strlen(params->ssid);

	params->psk = CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD;
	params->psk_length = strlen(params->psk);

	/* STEP 7.2 - Populate the rest of the relevant members */
	params->channel = WIFI_CHANNEL_ANY;
	params->security = WIFI_SECURITY_TYPE_PSK;
	params->mfp = WIFI_MFP_OPTIONAL;
	params->timeout = SYS_FOREVER_MS;
	params->band = WIFI_FREQ_BAND_UNKNOWN;
	memset(params->bssid, 0, sizeof(params->bssid));
	return 0;
}
void serial_cb(const struct device *dev, void *user_data);

const uint32_t pixel_format_table[] = {
	VIDEO_PIX_FMT_JPEG,
	VIDEO_PIX_FMT_RGB565,
	VIDEO_PIX_FMT_YUYV,
};

const uint16_t resolution_table[][2] = {
	{160, 120},  {320, 240},   {640, 480},   {800, 600},   {1280, 720},
	{1280, 960}, {1600, 1200}, {1920, 1080}, {2048, 1536}, {2592, 1944},
	{96, 96},    {128, 128},   {320, 320},
};

const uint8_t resolution_num = sizeof(resolution_table) / 4;

static uint8_t current_resolution;
static uint8_t take_picture_fmt = 0x1a;

int set_mega_resolution(uint8_t sfmt)
{
	uint8_t resolution = sfmt & 0x0f;
	uint8_t pixelformat = (sfmt & 0x70) >> 4;

	if (resolution > resolution_num || pixelformat > 3) {
		return -1;
	}
	struct video_format fmt = {.width = resolution_table[resolution][0],
				   .height = resolution_table[resolution][1],
				   .pixelformat = pixel_format_table[pixelformat - 1]};
	current_resolution = resolution;
	return video_set_format(video, VIDEO_EP_OUT, &fmt);
}

void uart_buffer_send(const struct device *dev, uint8_t *buffer, uint32_t length)
{
	for (uint32_t i = 0; i < length; i++) {
		uart_poll_out(dev, buffer[i]);
	}
}

static uint8_t head_and_tail[] = {0xff, 0xaa, 0x00, 0xff, 0xbb};

void uart_packet_send(uint8_t type, uint8_t *buffer, uint32_t length)
{
	head_and_tail[2] = type;
	uart_buffer_send(console, &head_and_tail[0], 3);
	uart_buffer_send(console, (uint8_t *)&length, 4);
	uart_buffer_send(console, buffer, length);
	uart_buffer_send(console, &head_and_tail[3], 2);
}

int take_picture(void)
{
	int err;
	enum video_frame_fragmented_status f_status;

	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
	if (err) {
		LOG_ERR("Unable to dequeue video buf");
		return -1;
	}

	f_status = vbuf->flags;

	head_and_tail[2] = 0x01;
	uart_buffer_send(console, &head_and_tail[0], 3);
	uart_buffer_send(console, (uint8_t *)&vbuf->bytesframe, 4);
	uart_poll_out(console, ((current_resolution & 0x0f) << 4) | 0x01);

	uart_buffer_send(console, vbuf->buffer, vbuf->bytesused);

	video_enqueue(video, VIDEO_EP_OUT, vbuf);
	while (f_status == VIDEO_BUF_FRAG) {
		video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
		f_status = vbuf->flags;
		uart_buffer_send(console, vbuf->buffer, vbuf->bytesused);
		video_enqueue(video, VIDEO_EP_OUT, vbuf);
	}
	uart_buffer_send(console, &head_and_tail[3], 2);

	return 0;
}

void video_preview(void)
{
	int err;
	enum video_frame_fragmented_status f_status;

	if (!preview_on) {
		return;
	}

	err = video_dequeue(video, VIDEO_EP_OUT, &vbuf, K_FOREVER);
	if (err) {
		LOG_ERR("Unable to dequeue video buf");
		return;
	}

	f_status = vbuf->flags;

	if (capture_flag == 1) {
		capture_flag = 0;
		head_and_tail[2] = 0x01;
		uart_buffer_send(console, &head_and_tail[0], 3);
		uart_buffer_send(console, (uint8_t *)&vbuf->bytesframe, 4);
		uart_poll_out(console, ((current_resolution & 0x0f) << 4) | 0x01);
	}

	uart_buffer_send(console, vbuf->buffer, vbuf->bytesused);

	if (f_status == VIDEO_BUF_EOF) {
		uart_buffer_send(console, &head_and_tail[3], 2);
		capture_flag = 1;
	}

	err = video_enqueue(video, VIDEO_EP_OUT, vbuf);
	if (err) {
		LOG_ERR("Unable to requeue video buf");
		return;
	}
}

int report_mega_info(void)
{
	char str_buf[400];
	uint32_t str_len;
	char *mega_type;
	struct arducam_mega_info mega_info;

	video_get_ctrl(video, VIDEO_CID_ARDUCAM_INFO, &mega_info);

	switch (mega_info.camera_id) {
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
	str_len = strlen(str_buf);
	uart_packet_send(0x02, str_buf, str_len);
	return 0;
}

uint8_t recv_process(uint8_t *buff)
{
	switch (buff[0]) {
	case SET_PICTURE_RESOLUTION:
		if (set_mega_resolution(buff[1]) == 0) {
			take_picture_fmt = buff[1];
		}
		break;
	case SET_VIDEO_RESOLUTION:
		if (preview_on == 0) {
			set_mega_resolution(buff[1] | 0x10);
			video_stream_start(video);
			capture_flag = 1;
		}
		preview_on = 1;
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
		take_picture();
		video_stream_stop(video);
		break;
	case STOP_STREAM:
		if (preview_on) {
			uart_buffer_send(console, &head_and_tail[3], 2);
			video_stream_stop(video);
			set_mega_resolution(take_picture_fmt);
		}
		preview_on = 0;
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

uint8_t uart_available(uint8_t *p)
{
	return k_msgq_get(&uart_msgq, p, K_NO_WAIT);
}

int main(void)
{

/* STEP 8.1 - Declare the variable for the network configuration parameters */
	struct wifi_connect_req_params cnx_params;

	/* STEP 8.2 - Get the network interface */
	struct net_if *iface = net_if_get_first_wifi();
	if (iface == NULL) {
		LOG_ERR("Returned network interface is NULL");
		return -1;
	}

	if (dk_leds_init() != 0) {
		LOG_ERR("Failed to initialize the LED library");
	}

	/* Sleep to allow initialization of Wi-Fi driver */
	k_sleep(K_SECONDS(1));

	/* STEP 9 - Initialize and add the callback function for network events */
	net_mgmt_init_event_callback(&mgmt_cb, net_mgmt_event_handler, EVENT_MASK);
	net_mgmt_add_event_callback(&mgmt_cb);

	/* STEP 10 - Populate cnx_params with the network configuration */
	wifi_args_to_params(&cnx_params);

	/* STEP 11 - Call net_mgmt() to request the Wi-Fi connection */
	LOG_INF("Connecting to Wi-Fi");
	int err = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &cnx_params, sizeof(struct wifi_connect_req_params));
	if (err) {
		LOG_ERR("Connecting to Wi-Fi failed, err: %d", err);
		return ENOEXEC;
	}

	 // **BLOCK here until Wi-Fi is connected**
    k_sem_take(&run_app, K_FOREVER);

	uint8_t recv_buffer[12] = {0};
	struct video_buffer *buffers[3];
	int i = 0;

	console = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
	if (!device_is_ready(console)) {
		LOG_ERR("%s: device not ready.", console->name);
		return -1;
	}
	uart_irq_callback_user_data_set(console, serial_cb, NULL);
	uart_irq_rx_enable(console);

	video = DEVICE_DT_GET(DT_NODELABEL(arducam0));

	if (!device_is_ready(video)) {
		LOG_ERR("Video device %s not ready.", video->name);
		return -1;
	}

 /* Initialize video buffers and enqueue */
    for (i = 0; i < NUM_BUFFERS; i++) {
        video_buffers[i].buffer  = dma_bufs[i];
        video_buffers[i].size =  DMA_BUF_SIZE;
        video_enqueue(video, VIDEO_EP_OUT, &video_buffers[i]);
    }

	LOG_INF("Mega star");

	printk("- Device name: %s\n", video->name);

	while (1) {
		if (!uart_available(recv_buffer)) {
			recv_process(recv_buffer);
		}
		video_preview();
		k_msleep(1);
	}
	return 0;
}

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

/*
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 */
void serial_cb(const struct device *dev, void *user_data)
{
	uint8_t c;

	if (!uart_irq_update(dev)) {
		return;
	}

	if (!uart_irq_rx_ready(dev)) {
		return;
	}

	/* read until FIFO empty */
	while (uart_fifo_read(dev, &c, 1) == 1) {
		if (c == 0xAA && rx_buf_pos > 0) {
			k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);
			rx_buf_pos = 0;
		} else if (c == 0x55) {
			rx_buf_pos = 0;
		} else {
			rx_buf[rx_buf_pos] = c;
			if (++rx_buf_pos >= MSG_SIZE) {
				rx_buf_pos = 0;
			}
		}
	}
}
