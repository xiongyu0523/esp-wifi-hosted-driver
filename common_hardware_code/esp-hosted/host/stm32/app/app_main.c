// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

/** Includes **/
#include "tx_api.h"
#include "spi_drv.h"
#include "control.h"
#include "trace.h"
#include "app_main.h"
#include "netdev_api.h"
#include "arp_server_stub.h"
#include "stm32l4xx_hal.h"

/** Constants/Macros **/
#define ARPING_PATH_TASK_STACK_SIZE     4096
#define ARPING_TASK_PRIO				0

/** Exported variables **/

/** Function declaration **/
static void init_sta(void);
static void init_ap(void);
static void reset_slave(void);
static void arping_task(ULONG arg);

static CHAR arping_task_stack[ARPING_PATH_TASK_STACK_SIZE];

struct network_handle *sta_handle, *ap_handle;
static TX_THREAD arping_task_id;

/** function definition **/

/** Local Functions **/
/**
  * @brief  Reset slave to initialize
  * @param  None
  * @retval None
  */
static void reset_slave(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* TODO: make this pin configurable from project config */
	GPIO_InitStruct.Pin = GPIO_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIO_RESET_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_RESET_GPIO_Port, GPIO_RESET_Pin, GPIO_PIN_RESET);
	hard_delay(50);

	/* revert to initial state */
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIO_RESET_GPIO_Port, &GPIO_InitStruct);

	/* stop spi transactions short time to avoid slave sync issues */
	hard_delay(50000);
}


/**
  * @brief  Control path event handler callback
  * @param  event - spi_drv_events_e event to be handled
  * @retval None
  */
static void control_path_event_handler(uint8_t event)
{
	switch(event)
	{
		case STATION_CONNECTED:
		{
			init_sta();
			break;
		}
		case STATION_DISCONNECTED:
		{
			printf("station disconnected\n\r");
			break;
		}
		case SOFTAP_STARTED:
		{
			init_ap();
			break;
		}
		case SOFTAP_STOPPED:
		{
			printf("softap stopped\n\r");
			break;
		}
		default:
		break;
	}
}

/**
  * @brief  SPI driver event handler callback
  * @param  event - spi_drv_events_e event to be handled
  * @retval None
  */
static void spi_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case SPI_DRIVER_ACTIVE:
		{
			/* Initiate control path now */
			control_path_init(control_path_event_handler);
			break;
		}
		default:
		break;
	}
}


/** Exported functions **/

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void Arping_test(void)
{
	UINT status;

	reset_slave();

	/* Init network interface */
	network_init();

	/* init spi driver */
	stm_spi_init(spi_driver_event_handler);

	/* This thread's priority shouls be >= spi driver's transaction task priority */
	status = tx_thread_create(&arping_task_id, "arping_task", 
							  arping_task, 0, arping_task_stack, ARPING_PATH_TASK_STACK_SIZE,
							  ARPING_TASK_PRIO, ARPING_TASK_PRIO, TX_NO_TIME_SLICE, TX_AUTO_START);
	assert(status == TX_SUCCESS);
}

/**
  * @brief Station mode rx callback
  * @param  net_handle - station network handle
  * @retval None
  */
static void sta_rx_callback(struct network_handle *net_handle)
{
	struct pbuf *rx_buffer = NULL;
	struct pbuf *snd_buffer = NULL;
	uint8_t *arp_resp = NULL;
	uint16_t arp_resp_len = 0;
	uint32_t sta_ip = 0;
	int ret;

	rx_buffer = network_read(net_handle, 0);

	if (get_self_ip_station(&sta_ip)) {
		printf("Problem getting self station ip\n\r");
		if(rx_buffer) {
			free(rx_buffer->payload);
			rx_buffer->payload = NULL;
			free(rx_buffer);
			rx_buffer = NULL;
		}
		return;
	}

	if (rx_buffer) {
		arp_resp = arp_req_handler(&sta_ip, get_self_mac_station(), rx_buffer->payload,
				rx_buffer->len, &arp_resp_len);

		if (arp_resp) {
			snd_buffer = malloc(sizeof(struct pbuf));
			assert(snd_buffer);

			snd_buffer->payload = arp_resp;
			snd_buffer->len = arp_resp_len;

			ret = network_write(net_handle, snd_buffer);

			if (ret)
				printf("%s: Failed to send arp response\n\r", __func__);
		}

		free(rx_buffer->payload);
		rx_buffer->payload = NULL;
		free(rx_buffer);
		rx_buffer = NULL;
	}
}

/**
  * @brief Softap mode rx callback
  * @param  net_handle - Softap network handle
  * @retval None
  */
static void ap_rx_callback(struct network_handle *net_handle)
{
	struct pbuf *rx_buffer = NULL;
	struct pbuf *snd_buffer = NULL;
	uint8_t *arp_resp = NULL;
	uint16_t arp_resp_len = 0;
	int ret;
	uint32_t softap_ip = 0;

	rx_buffer = network_read(net_handle, 0);

	if (get_self_ip_softap(&softap_ip)) {
		printf("Problem getting self softap ip\n\r");
		if(rx_buffer) {
			free(rx_buffer->payload);
			rx_buffer->payload = NULL;
			free(rx_buffer);
			rx_buffer = NULL;
		}
		return;
	}

	if (rx_buffer) {
		arp_resp = arp_req_handler(&softap_ip, get_self_mac_softap(),
				rx_buffer->payload, rx_buffer->len, &arp_resp_len);

		if (arp_resp) {
			snd_buffer = malloc(sizeof(struct pbuf));
			assert(snd_buffer);

			snd_buffer->payload = arp_resp;
			snd_buffer->len = arp_resp_len;

			ret = network_write(net_handle, snd_buffer);

			if (ret)
				printf("%s: Failed to send arp response\n\r", __func__);
		}

		free(rx_buffer->payload);
		rx_buffer->payload = NULL;
		free(rx_buffer);
		rx_buffer = NULL;
	}
}


/**
  * @brief start station mode network path
  * @param None
  * @retval None
  */
static void init_sta(void)
{
	sta_handle = network_open(STA_INTERFACE, sta_rx_callback);
	assert(sta_handle);
}

/**
  * @brief start softap mode network path
  * @param None
  * @retval None
  */
static void init_ap(void)
{
	ap_handle = network_open(SOFTAP_INTERFACE, ap_rx_callback);
	assert(ap_handle);
}

/**
  * @brief task initiate arping req periodically
  * @param Not used
  * @retval None
  */
static void arping_task(ULONG arg)
{
	uint32_t sta_ip, softap_ip;
	uint32_t sta_dest_ip, softap_dest_ip;
	uint8_t  dst_mac_bytes[MAC_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	sta_ip = softap_ip = sta_dest_ip = softap_dest_ip = 0;

	get_self_ip_station(&sta_ip);
	get_self_ip_softap(&softap_ip);
	get_arp_dst_ip_station(&sta_dest_ip);
	get_arp_dst_ip_softap(&softap_dest_ip);

	while (1) {
		if (sta_handle)
			send_arp_req(sta_handle, get_self_mac_station(), &sta_ip, dst_mac_bytes, &sta_dest_ip);

		if(ap_handle)
			send_arp_req(ap_handle, get_self_mac_softap(), &softap_ip, dst_mac_bytes, &softap_dest_ip);

		tx_thread_sleep(MS_TO_TICKS(1000));
	}
}
