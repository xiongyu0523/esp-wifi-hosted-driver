#include "nx_api.h"
#include "nx_arp.h"
#include "nx_rarp.h"

#include "spi_drv.h"
#include "netdev_api.h"
#include "commands.h"
#include "common.h"
#include "stm32l4xx_hal.h"

#include "nx_driver_esp_hosted.h"

/* Define the Link MTU. Note this is not the same as the IP MTU. The Link MTU
   includes the addition of the Physical Network header (usually Ethernet). This
   should be larger than the IP instance MTU by the size of the physical header. */
#define NX_LINK_MTU             1514
#define NX_ETHERNET_SIZE        14
#define NX_ETHERNET_IP          0x0800
#define NX_ETHERNET_ARP         0x0806
#define NX_ETHERNET_RARP        0x8035
#define NX_ETHERNET_IPV6        0x86DD
#define NX_DRIVER_ERROR         90

#define MAC_STR_LEN             18
#define NX_DRIVER_JOIN_MAX_CNT  3

#define NX_DRIVER_ETHERNET_HEADER_REMOVE(p)   \
{   \
    p -> nx_packet_prepend_ptr =  p -> nx_packet_prepend_ptr + NX_ETHERNET_SIZE;  \
    p -> nx_packet_length =  p -> nx_packet_length - NX_ETHERNET_SIZE;    \
} 

static NX_PACKET_POOL   *_pool_ptr = NULL;
static NX_IP            *_ip_ptr   = NULL;

static TX_SEMAPHORE     _device_ready_semphr;
struct network_handle   *sta_handle;

UCHAR   _nx_driver_hardware_address[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x56};  

/* Define the routines for processing each driver entry request */
static UINT _nx_driver_initialize(NX_IP_DRIVER *driver_req_ptr);
static UINT _nx_driver_uninitialize(NX_IP_DRIVER *driver_req_ptr);
static UINT _nx_driver_enable(NX_IP_DRIVER *driver_req_ptr);
static UINT _nx_driver_disable(NX_IP_DRIVER *driver_req_ptr);
static UINT _nx_driver_packet_send(NX_IP_DRIVER *driver_req_ptr);

/* Define the prototypes for the hardware implementation of this driver */
static void _nx_driver_transfer_to_netx(NX_IP *ip_ptr, NX_PACKET *packet_ptr);

/**************************************************************************//**
 * Network driver entry function
 *****************************************************************************/
void nx_driver_esp_hosted(NX_IP_DRIVER *driver_req_ptr)
{
    UINT error_code = NX_SUCCESS;

    /* Default to successful return */
    driver_req_ptr->nx_ip_driver_status = NX_SUCCESS;

    /* Process according to the driver request type in the IP control block */
    switch (driver_req_ptr->nx_ip_driver_command)
    {
    case NX_LINK_INTERFACE_ATTACH:
        /* Process driver link attach. Unsupported feature */
        break;

    case NX_LINK_INTERFACE_DETACH:
        /* Process driver link detach. Unsupported feature */
        break;

    case NX_LINK_INITIALIZE:
        /* Process driver link initialize */
        error_code = _nx_driver_initialize(driver_req_ptr);
        break;

    case NX_LINK_UNINITIALIZE:
        /* Process driver link uninitialize */
        error_code = _nx_driver_uninitialize(driver_req_ptr);
        break;

    case NX_LINK_ENABLE:
        /* Process driver link enable */
        error_code = _nx_driver_enable(driver_req_ptr);
        break;

    case NX_LINK_DISABLE:
        /* Process driver link disable */
        error_code = _nx_driver_disable(driver_req_ptr);
        break;

    case NX_LINK_PACKET_SEND:
    case NX_LINK_PACKET_BROADCAST:
    case NX_LINK_ARP_SEND:
    case NX_LINK_ARP_RESPONSE_SEND:
    case NX_LINK_RARP_SEND:
        /* Process packet send requests */
        error_code = _nx_driver_packet_send(driver_req_ptr);
        break;

    case NX_LINK_MULTICAST_JOIN:
        /* Process driver multicast join. Unsupported feature */
        break;

    case NX_LINK_MULTICAST_LEAVE:
        /* Process driver multicast leave. Unsupported feature */
        break;

    case NX_LINK_GET_STATUS:
        /* Process driver get link status. Unsupported feature */
        break;

    case NX_LINK_GET_SPEED:
        /* Return the link's line speed in the supplied return pointer. Unsupported feature */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = 0;
        break;

    case NX_LINK_GET_DUPLEX_TYPE:
        /* Return the link's line speed in the supplied return pointer. Unsupported feature */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = 0;
        break;

    case NX_LINK_GET_ERROR_COUNT:
        /* Return the link's line speed in the supplied return pointer. Unsupported feature */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = 0;
        break;

    case NX_LINK_GET_RX_COUNT:
        /* Return the link's line speed in the supplied return pointer. Unsupported feature */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = 0;
        break;

    case NX_LINK_GET_TX_COUNT:
        /* Return the link's line speed in the supplied return pointer. Unsupported feature */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = 0;
        break;

    case NX_LINK_GET_ALLOC_ERRORS:
        /* Return the link's line speed in the supplied return pointer. Unsupported feature */
        *(driver_req_ptr->nx_ip_driver_return_ptr) = 0;
        break;

    case NX_LINK_DEFERRED_PROCESSING:
        /* Process driver link deferred processing. Unsupported feature */
        break;

    case NX_LINK_SET_PHYSICAL_ADDRESS:
        /* Process driver link set physical address. Unsupported feature */
        break;

    default:
        /* Return the unhandled command status */
        driver_req_ptr->nx_ip_driver_status = NX_UNHANDLED_COMMAND;
        break;
    }

    if (error_code != NX_SUCCESS) {
        driver_req_ptr->nx_ip_driver_status = NX_NOT_SUCCESSFUL;
    }
}

/**************************************************************************//**
 * Receive incoming packet from ESP32
 *****************************************************************************/
void _nx_driver_receive_callback(struct pbuf *rx_buffer)
{
    NX_PACKET *packet_ptr;
    UCHAR     *packet_buffer;
    UINT      status;

    /* Allocate a NX_PACKET to be passed to the IP stack */
    status = nx_packet_allocate(_pool_ptr, &packet_ptr, NX_IP_PACKET, TX_WAIT_FOREVER);
    if (status != NX_SUCCESS) {
        printf("_nx_driver_receive_callback: unable to allocate memory for receive packet\n");
        return;
    }

    packet_buffer = rx_buffer->payload;
    /* Setup the ethernet frame pointer to build the ethernet frame. Backup another 2
        bytes to get 32-bit word alignment. */
    packet_buffer = packet_buffer - 2;
    status = nx_packet_data_append(packet_ptr, packet_buffer,rx_buffer->len + 2, _pool_ptr, TX_WAIT_FOREVER);
    if (status != NX_SUCCESS) {
        printf("_nx_driver_receive_callback: packet append error\n");
        return;
    }

    /* Clean off the offset */
    packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + 2;

    /* Adjust the packet length */
    packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - 2;

    _nx_driver_transfer_to_netx(_ip_ptr, packet_ptr);
}

/**************************************************************************//**
 * Handle the spi driver event
 *****************************************************************************/
static void _reset_slave(void)
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

/**************************************************************************//**
 * Handle the spi driver event
 *****************************************************************************/
static void _spi_driver_event_handler(uint8_t event)
{
	switch(event)
	{
		case SPI_DRIVER_ACTIVE:
		{
            (VOID)tx_semaphore_put(&_device_ready_semphr);
			break;
		}
		default:
		break;
	}
}

/**************************************************************************//**
 * Handle the sta rx event
 *****************************************************************************/
static void _sta_rx_callback(struct network_handle *net_handle)
{
	struct pbuf *rx_buffer = NULL;
	
	rx_buffer = network_read(net_handle, 0);
	if (rx_buffer) {

        _nx_driver_receive_callback(rx_buffer);

		free(rx_buffer->payload);
		rx_buffer->payload = NULL;
		free(rx_buffer);
		rx_buffer = NULL;
	}
}

/**************************************************************************//**
 * Handle the driver initialize request
 *****************************************************************************/
static UINT _nx_driver_initialize(NX_IP_DRIVER *driver_req_ptr)
{
    UINT    interface_index;
    ULONG   mac_addr_lsw, mac_addr_msw;
    UINT    error_code = NX_SUCCESS;
	char    mac[MAC_STR_LEN];

    /* Setup the IP pointer from the driver request */
    _ip_ptr = driver_req_ptr->nx_ip_driver_ptr;

    /* Obtain the index number of the network interface */
    interface_index = driver_req_ptr->nx_ip_driver_interface->nx_interface_index;

    /* Setup the default packet pool for the driver's received packets */
    _pool_ptr = _ip_ptr->nx_ip_default_packet_pool;

    tx_semaphore_create(&_device_ready_semphr, "_device_ready_semphr", 0);

    _reset_slave();
    network_init();
    stm_spi_init(_spi_driver_event_handler);

    tx_semaphore_get(&_device_ready_semphr, TX_WAIT_FOREVER);

    control_path_platform_init();
	memset(mac, 0, MAC_STR_LEN);
	wifi_get_mac(WIFI_MODE_STA, mac);

    /* Once the Ethernet controller is initialized, the driver needs to
        configure the NetX Interface Control block, as outlined below */

    /* The nx_interface_ip_mtu_size should be the MTU for the IP payload.
        For regular Ethernet, the IP MTU is 1500 */
    nx_ip_interface_mtu_set(_ip_ptr, interface_index, (NX_LINK_MTU - NX_ETHERNET_SIZE));

    /* Set the physical address (MAC address) of this IP instance */
    mac_addr_msw = (ULONG)((mac[0] << 8)
                            + mac[1]);
    mac_addr_lsw = (ULONG)((mac[2] << 24)
                            + (mac[3] << 16)
                            + (mac[4] << 8)
                            + (mac[5]));

    nx_ip_interface_physical_address_set(_ip_ptr, interface_index,
                                        mac_addr_msw, mac_addr_lsw, NX_FALSE);

    /* Indicate to the IP software that IP to physical mapping is required */
    nx_ip_interface_address_mapping_configure(_ip_ptr, interface_index, NX_TRUE);
    return error_code;
}

/**************************************************************************//**
 * Handle the link enable request
 *****************************************************************************/
static UINT _nx_driver_enable(NX_IP_DRIVER *driver_req_ptr)
{
    UINT error_code = NX_DRIVER_ERROR;
    UINT retry_count = 0;
    esp_hosted_control_config_t ap_config;

    /* Get the wifi connection info from application */
    nx_wifi_info_t *wifi_info_ptr = (nx_wifi_info_t *)driver_req_ptr->nx_ip_driver_interface->nx_interface_additional_link_info;

	strncpy((char* )&ap_config.station.ssid, wifi_info_ptr->ssid, SSID_LENGTH);
	strncpy((char* )&ap_config.station.pwd, wifi_info_ptr->password,PASSWORD_LENGTH);
    ap_config.station.encryption_mode = wifi_info_ptr->mode;
	ap_config.station.is_wpa3_supported = 0;

    do {
        
        retry_count++;

        printf("%d try to connect to SSID: %s\n", retry_count, wifi_info_ptr->ssid);

        if (wifi_set_ap_config(ap_config) == SUCCESS) {
            printf("Connection established\r\n");

            sta_handle = network_open(STA_INTERFACE, _sta_rx_callback);
            if (sta_handle != NULL) {
                error_code = SUCCESS;
            }
            
            break;
        } else {
            if (retry_count > NX_DRIVER_JOIN_MAX_CNT) {
                break;
            } else {
                tx_thread_sleep(100);
            }
        }
    } while (1);

    if (error_code == NX_SUCCESS) {
        driver_req_ptr->nx_ip_driver_interface->nx_interface_link_up = NX_TRUE;
    }

    return error_code;
}

/**************************************************************************//**
 * Handle the driver packet send request
 *****************************************************************************/
static UINT _nx_driver_packet_send(NX_IP_DRIVER *driver_req_ptr)
{
    NX_PACKET    *packet_ptr;
    ULONG        *ethernet_frame_ptr;
    NX_INTERFACE *interface_ptr;

    /* Setup interface pointer */
    interface_ptr = driver_req_ptr->nx_ip_driver_interface;

    /* Place the ethernet frame at the front of the packet */
    packet_ptr = driver_req_ptr->nx_ip_driver_packet;

    /* Adjust the prepend pointer */
    packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr - NX_ETHERNET_SIZE;

    /* Adjust the packet length */
    packet_ptr->nx_packet_length = packet_ptr->nx_packet_length + NX_ETHERNET_SIZE;

    /* Setup the ethernet frame pointer to build the ethernet frame. Backup another 2
        bytes to get 32-bit word alignment. */
    /*lint -e{927} -e{826} suppress cast of pointer to pointer, since it is necessary */
    ethernet_frame_ptr = (ULONG *)(packet_ptr->nx_packet_prepend_ptr - 2);

    /* Build the ethernet frame */
    *ethernet_frame_ptr       = driver_req_ptr->nx_ip_driver_physical_address_msw;
    *(ethernet_frame_ptr + 1) = driver_req_ptr->nx_ip_driver_physical_address_lsw;
    *(ethernet_frame_ptr + 2) = (interface_ptr->nx_interface_physical_address_msw << 16)
                                | (interface_ptr->nx_interface_physical_address_lsw >> 16);
    *(ethernet_frame_ptr + 3) = (interface_ptr->nx_interface_physical_address_lsw << 16);

    if (driver_req_ptr->nx_ip_driver_command == NX_LINK_ARP_SEND) {
        *(ethernet_frame_ptr + 3) |= NX_ETHERNET_ARP;
    } else if (driver_req_ptr->nx_ip_driver_command == NX_LINK_ARP_RESPONSE_SEND) {
        *(ethernet_frame_ptr + 3) |= NX_ETHERNET_ARP;
    } else if (driver_req_ptr->nx_ip_driver_command == NX_LINK_RARP_SEND) {
        *(ethernet_frame_ptr + 3) |= NX_ETHERNET_RARP;
    } else if (packet_ptr->nx_packet_ip_version == 4) {
        *(ethernet_frame_ptr + 3) |= NX_ETHERNET_IP;
    } else {
        *(ethernet_frame_ptr + 3) |= NX_ETHERNET_IPV6;
    }

    /* Endian swapping if NX_LITTLE_ENDIAN is defined */
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr));
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 1));
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 2));
    NX_CHANGE_ULONG_ENDIAN(*(ethernet_frame_ptr + 3));

    for (NX_PACKET *p = packet_ptr; p != NULL; p = p->nx_packet_next) {

        struct pbuf *buffer = malloc(sizeof(struct pbuf));
		if (buffer != NULL) {
			buffer->len = p->nx_packet_length;
			buffer->payload = malloc(buffer->len);
			if (buffer->payload != NULL) {
				memcpy(buffer->payload, p->nx_packet_prepend_ptr, buffer->len);    
				network_write(sta_handle, buffer);
			}
		}
    }

    NX_DRIVER_ETHERNET_HEADER_REMOVE(packet_ptr);
    nx_packet_transmit_release(packet_ptr);

    return NX_SUCCESS;
}


/**************************************************************************//**
 * Handle the driver uninitialize request
 *****************************************************************************/
static UINT _nx_driver_uninitialize(NX_IP_DRIVER *driver_req_ptr)
{
    return NX_SUCCESS;
}

/**************************************************************************//**
 * Handle the driver disable request
 *****************************************************************************/
static UINT _nx_driver_disable(NX_IP_DRIVER *driver_req_ptr)
{
    driver_req_ptr->nx_ip_driver_interface->nx_interface_link_up = NX_FALSE;
    return NX_SUCCESS;
}

/**************************************************************************//**
 * Transfer packet received from ESP32 to IP stack
 *****************************************************************************/
static void _nx_driver_transfer_to_netx(NX_IP *ip_ptr, NX_PACKET *packet_ptr)
{
    UINT packet_type;

    /* Pickup the packet header to determine where the packet needs to be sent */
    packet_type = (((UINT)(*(packet_ptr->nx_packet_prepend_ptr + 12))) << 8)
                    | ((UINT)(*(packet_ptr->nx_packet_prepend_ptr + 13)));
    /* Setup interface pointer */
    packet_ptr->nx_packet_address.nx_packet_interface_ptr = &ip_ptr->nx_ip_interface[0];

    /* Route the incoming packet according to its ethernet type */
    if ((packet_type == NX_ETHERNET_IP) || (packet_type == NX_ETHERNET_IPV6)) {

        /* Note: The length reported by some Ethernet hardware includes bytes after the packet
            as well as the Ethernet header. In some cases, the actual packet length after the
            Ethernet header should be derived from the length in the IP header (lower 16 bits of
            the first 32-bit word). */

        /* Clean off the Ethernet header */
        packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

        /* Adjust the packet length */
        packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - NX_ETHERNET_SIZE;

        /* Route to the ip receive function */
        _nx_ip_packet_receive(ip_ptr, packet_ptr);
    }
#ifndef NX_DISABLE_IPV4
    else if (packet_type == NX_ETHERNET_ARP) {
            /* Clean off the Ethernet header */
            packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

            /* Adjust the packet length */
            packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - NX_ETHERNET_SIZE;

            /* Route to the ARP receive function */
            _nx_arp_packet_receive(ip_ptr, packet_ptr);
        } else if (packet_type == NX_ETHERNET_RARP) {
            /* Clean off the Ethernet header */
            packet_ptr->nx_packet_prepend_ptr = packet_ptr->nx_packet_prepend_ptr + NX_ETHERNET_SIZE;

            /* Adjust the packet length */
            packet_ptr->nx_packet_length = packet_ptr->nx_packet_length - NX_ETHERNET_SIZE;
            /* Route to the RARP receive function */
            _nx_rarp_packet_receive(ip_ptr, packet_ptr);
    }
#endif /* !NX_DISABLE_IPV4 */
    else {
        /* Invalid ethernet header... release the packet */
        nx_packet_release(packet_ptr);
    }
}
