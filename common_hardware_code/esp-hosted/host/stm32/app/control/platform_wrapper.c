// Copyright 2015-2021 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//#include "unistd.h"

#include "tx_api.h"

#include "string.h"

#include "trace.h"
#include "serial_if.h"
#include "transport_pserial.h"
#include "platform_wrapper.h"

#define BYTE_POOL_SIZE			(64 * 1024)

static TX_SEMAPHORE readSemaphore;
static serial_handle_t * serial_if_g;

static UCHAR        byte_pool_mem[BYTE_POOL_SIZE];
static TX_BYTE_POOL byte_tool;

static void control_path_rx_indication(void);

struct esp_hosted_driver_handle_t {
    int handle; /* dummy variable */
};

int control_path_platform_init(void)
{
	UINT status;

	/* control path semaphore */
	status = tx_semaphore_create(&readSemaphore, "readSemaphore", 1);
	assert(status == TX_SUCCESS);

	/* grab the semaphore, so that task will be mandated to wait on semaphore */
	if (tx_semaphore_get(&readSemaphore, TX_WAIT_FOREVER) != TX_SUCCESS) {
	    printf("could not obtain readSemaphore\n\r");
	    return STM_FAIL;
	}

	serial_if_g = serial_init(control_path_rx_indication);
	if (!serial_if_g) {
	    printf("Serial interface creation failed\n\r");
	    assert(serial_if_g);
	    return STM_FAIL;
	}
	if (STM_OK != serial_if_g->fops->open(serial_if_g)) {
		printf("Serial interface open failed\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

int control_path_platform_deinit(void)
{
	if (STM_OK != serial_if_g->fops->close(serial_if_g)) {
		printf("Serial interface close failed\n\r");
		return STM_FAIL;
	}
	return STM_OK;
}

static void control_path_rx_indication(void)
{
	(VOID)tx_semaphore_put(&readSemaphore);
}

void esp_hosted_heap_init(void)
{
    (VOID)tx_byte_pool_create(&byte_tool, "ESP Hosted Byte Pool", byte_pool_mem, BYTE_POOL_SIZE);
}

void* esp_hosted_malloc(size_t size)
{
    void *pointer;
    return tx_byte_allocate(&byte_tool, &pointer, size, TX_NO_WAIT) == TX_SUCCESS ? pointer : NULL;
}

void* esp_hosted_calloc(size_t blk_no, size_t size)
{
    size_t total = blk_no * size;
    void *pointer = esp_hosted_malloc(total);
    TX_MEMSET(pointer, 0, total);
    return pointer;
}

void esp_hosted_free(void* ptr)
{
    (VOID)tx_byte_release(ptr);
}

struct esp_hosted_driver_handle_t* esp_hosted_driver_open(const char* transport)
{
	struct esp_hosted_driver_handle_t* esp_hosted_driver_handle = NULL;
	if (!transport) {
		printf("Invalid parameter in open \n\r");
		return NULL;
	}
	esp_hosted_driver_handle = (struct esp_hosted_driver_handle_t*) esp_hosted_calloc
		(1,sizeof(struct esp_hosted_driver_handle_t));
	if (!esp_hosted_driver_handle) {
		printf("Failed to allocate memory \n");
		return NULL;
	}
	return esp_hosted_driver_handle;
}

int esp_hosted_driver_write (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle,
	uint8_t* buf, int in_count, int* out_count)
{
	int ret = 0;
	if (!esp_hosted_driver_handle || !buf || !in_count || !out_count) {
		printf("Invalid parameters in write\n\r");
		return STM_FAIL;
	}

	if( (!serial_if_g) ||
		(!serial_if_g->fops) ||
		(!serial_if_g->fops->write)) {
		printf("serial interface not valid\n\r");
		return STM_FAIL;
	}

	ret = serial_if_g->fops->write(serial_if_g, buf, in_count);
	if (ret != STM_OK) {
		*out_count = 0;
		printf("Failed to write data\n\r");
		return STM_FAIL;
	}

	*out_count = in_count;
	return STM_OK;
}

uint8_t* esp_hosted_driver_read (struct esp_hosted_driver_handle_t* esp_hosted_driver_handle,
	int read_len, uint8_t wait, uint32_t* buf_len)
{
	uint16_t rx_buf_len = 0;
	uint8_t* read_buf = NULL;
	uint8_t* buf = NULL;
	int len = 0, ret = 0;

	if (!esp_hosted_driver_handle || !read_len || !buf_len || !wait) {
		printf("Invalid parameters in read\n\r");
		return NULL;
	}
	if(readSemaphore.tx_semaphore_id == TX_CLEAR_ID) {
		printf("Semaphore not initialized\n\r");
		return NULL;
	}
	if (tx_semaphore_get(&readSemaphore , MS_TO_TICKS(wait * 1000)) != TX_SUCCESS) {
		printf("Failed to read data \n\r");
		return NULL;
	}

	if( (!serial_if_g) ||
		(!serial_if_g->fops) ||
		(!serial_if_g->fops->read)) {
		printf("serial interface refusing to read\n\r");
		return NULL;
	}

	read_buf = serial_if_g->fops->read(serial_if_g, &rx_buf_len);
	if ((! read_buf) || (!rx_buf_len)) {
		printf("serial read failed\n\r");
		return NULL;
	}
	print_hex_dump(read_buf, rx_buf_len, "Serial read data");

/*
 * Read Operation happens in two steps because total read length is unkown
 * at first read.
 *      1) Read fixed length of RX data
 *      2) Read variable length of RX data
 *
 * 1) Read fixed length of RX data :
 * Read fixed length of received data in below format:
 * ----------------------------------------------------------------------------
 *  Endpoint Type | Endpoint Length | Endpoint Value  | Data Type | Data Length
 * ----------------------------------------------------------------------------
 *
 *  Bytes used per field as follows:
 *  ---------------------------------------------------------------------------
 *      1         |       2         | Endpoint Length |     1     |     2     |
 *  ---------------------------------------------------------------------------
 *
 *  read_len = 1 + 2 + Endpoint length + 1 + 2
 */

	len = min(rx_buf_len,read_len);
	buf = (uint8_t* )esp_hosted_calloc(1, len);
	if (!buf) {
		printf("Failed to allocate memory \n\r");
		goto err;
	}

	memset(buf, 0, read_len);
	memcpy(buf, read_buf, len);

 /* parse_tlv function returns variable payload length of received data in buf_len */
	ret = parse_tlv(buf, buf_len);
	if (ret != STM_OK) {
		esp_hosted_free(buf);
		printf("Failed to parse RX data \n\r");
		goto err;
	}

	esp_hosted_free(buf);

	buf = NULL;
/*
 * 2) Read variable length of RX data:
 */
	buf = (uint8_t* )esp_hosted_calloc(1, *buf_len);
	if (!buf) {
		printf("Failed to allocate memory \n\r");
		goto err;
	}

	memcpy((buf), read_buf+read_len, *buf_len);

	esp_hosted_free(read_buf);
	read_buf = NULL;
	return buf;
err:
	if (read_buf) {
		esp_hosted_free(read_buf);
	}
	return NULL;
}

int esp_hosted_driver_close(struct esp_hosted_driver_handle_t** esp_hosted_driver_handle)
{
	if (!esp_hosted_driver_handle || !(*esp_hosted_driver_handle)) {
		printf("Invalid parameter in close \n\r");
	}
	esp_hosted_free(*esp_hosted_driver_handle);
	return STM_OK;
}
