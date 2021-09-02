/* Includes ------------------------------------------------------------------*/
#include "lwesp/lwesp.h"
#include "lwesp/lwesp_sta.h"
#include "lwesp/lwesp_dns.h"
#include "lwesp/lwesp_netconn.h"
#include "wifi.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

static lwespr_t lwesp_callback_func(lwesp_evt_t* evt) {
    switch (lwesp_evt_get_type(evt)) {
        case LWESP_EVT_AT_VERSION_NOT_SUPPORTED: {
            lwesp_sw_version_t v_min, v_curr;

            lwesp_get_min_at_fw_version(&v_min);
            lwesp_get_current_at_fw_version(&v_curr);

            printf("Current ESP AT version is not supported by library!\r\n");
            printf("Minimum required AT version is: %d.%d.%d\r\n", (int)v_min.major, (int)v_min.minor, (int)v_min.patch);
            printf("Current AT version is: %d.%d.%d\r\n", (int)v_curr.major, (int)v_curr.minor, (int)v_curr.patch);
            break;
        }
        case LWESP_EVT_INIT_FINISH: {
            printf("Library initialized!\r\n");
            break;
        }
        case LWESP_EVT_RESET_DETECTED: {
            printf("Device reset detected!\r\n");
            break;
        }
        case LWESP_EVT_WIFI_IP_ACQUIRED: {
            printf("IP acquired\r\n");
            break;
        }
        case LWESP_EVT_AP_CONNECTED_STA: {
            lwesp_mac_t* mac = lwesp_evt_ap_connected_sta_get_mac(evt);
            printf("Station %02X:%02X:%02X:%02X:%02X:%02X connected\r\n", mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
            break;
        }
        case LWESP_EVT_AP_IP_STA: {
            lwesp_mac_t* mac = lwesp_evt_ap_ip_sta_get_mac(evt);
            lwesp_ip_t* ip = lwesp_evt_ap_ip_sta_get_ip(evt);

            printf("Assigned %d.%d.%d.%d to %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                   ip->addr.ip4.addr[0], ip->addr.ip4.addr[1], ip->addr.ip4.addr[2], ip->addr.ip4.addr[3],
                   mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
            break;
        }
        case LWESP_EVT_AP_DISCONNECTED_STA: {
            lwesp_mac_t* mac = lwesp_evt_ap_disconnected_sta_get_mac(evt);
            printf("Station %02X:%02X:%02X:%02X:%02X:%02X disconnected\r\n", mac->mac[0], mac->mac[1], mac->mac[2], mac->mac[3], mac->mac[4], mac->mac[5]);
            break;
        }

        default: break;
    }
    return lwespOK;
}

/* Public functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the WIFI core
  * @param  None
  * @retval Operation status
  */
WIFI_Status_t WIFI_Init(void)
{
    lwespr_t api_ret = lwespERR;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_init(lwesp_callback_func, 1);
    if (api_ret == lwespOK) {
        api_ret = lwesp_set_wifi_mode(LWESP_MODE_STA_AP, NULL, NULL, 1);
        if (api_ret == lwespOK) {
          ret = WIFI_STATUS_OK;
        }
    }
    
    return ret;
}

/**
  * @brief  List a defined number of available access points
  * @param  APs : pointer to APs structure
  * @param  AP_MaxNbr : Max APs number to be listed
  * @retval Operation status
  */
WIFI_Status_t WIFI_ListAccessPoints(WIFI_APs_t *APs, uint8_t AP_MaxNbr)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Join an Access Point
  * @param  SSID : SSID string
  * @param  Password : Password string
  * @param  ecn : Encryption type
  * @retval Operation status
  */
WIFI_Status_t WIFI_Connect(
                             const char* SSID,
                             const char* Password,
                             WIFI_Ecn_t ecn)
{
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;
    (void)ecn;

    api_ret = lwesp_sta_join(SSID, Password, NULL, NULL, NULL, 1); 
    if (api_ret == lwespOK) {
        ret = WIFI_STATUS_OK;
    }

    return ret;
}

/**
  * @brief  This function retrieves the WiFi interface's MAC address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetMAC_Address(uint8_t  *mac)
{
    lwesp_mac_t mac_addr;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_sta_getmac(&mac_addr, NULL, NULL, 1);
    if (api_ret == lwespOK) {
        (void)memcpy(mac, &mac_addr.mac[0], 6);
        ret = WIFI_STATUS_OK;
    }

    return ret;
}
    
/**
  * @brief  This function retrieves the WiFi interface's IP address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetIP_Address (uint8_t  *ipaddr)
{
    lwesp_ip_t ip_addr;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    if (lwesp_sta_is_joined()) {
        api_ret = lwesp_sta_getip(&ip_addr, NULL, NULL, NULL, NULL, 1);
        if (api_ret == lwespOK) {
            if (ipaddr != NULL) {
                memcpy(ipaddr, &ip_addr.addr.ip4.addr[0], 4);
                ret = WIFI_STATUS_OK;
            }
        }
    }

    return ret;
}

/**
  * @brief  This function retrieves the WiFi interface's IP mask.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetIP_Mask (uint8_t  *mask)
{
    lwesp_ip_t nm_addr;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    if (lwesp_sta_is_joined()) {
        api_ret = lwesp_sta_getip(NULL, NULL, &nm_addr, NULL, NULL, 1);
        if (api_ret == lwespOK) {
            if (mask != NULL) {
                memcpy(mask, &nm_addr.addr.ip4.addr[0], 4);
                ret = WIFI_STATUS_OK;
            }
        }
    }

    return ret;
}

/**
  * @brief  This function retrieves the WiFi interface's Gateway address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetGateway_Address (uint8_t  *Gateway_addr)
{
    lwesp_ip_t gw_addr;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    if (lwesp_sta_is_joined()) {
        api_ret = lwesp_sta_getip(NULL, &gw_addr, NULL, NULL, NULL, 1);
        if (api_ret == lwespOK) {
            if (Gateway_addr != NULL) {
                memcpy(Gateway_addr, &gw_addr.addr.ip4.addr[0], 4);
                ret = WIFI_STATUS_OK;
            }
        }
    }

    return ret;
}

/**
  * @brief  This function retrieves the WiFi interface's DNS address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetDNS_Address (uint8_t  *DNS1addr,uint8_t  *DNS2addr)
{
    lwesp_ip_t dns1_addr, dns2_addr;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    if (lwesp_sta_is_joined()) {
        api_ret = lwesp_dns_get_config(&dns1_addr, &dns2_addr, NULL, NULL, 1);
        if (api_ret == lwespOK) {
            if ((DNS1addr != NULL) && (DNS2addr != NULL)) {
                memcpy(DNS1addr, &dns1_addr.addr.ip4.addr[0], 4);
                memcpy(DNS2addr, &dns2_addr.addr.ip4.addr[0], 4);
                ret = WIFI_STATUS_OK;
            }
        }
    }

    return ret;
}

/**
  * @brief  Disconnect from a network
  * @param  None
  * @retval Operation status
  */
WIFI_Status_t WIFI_Disconnect(void)
{
    lwespr_t api_ret = lwespERR;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_sta_quit(NULL, NULL, 1); 
    if (api_ret == lwespOK) {
        ret = WIFI_STATUS_OK;
    }

    return ret;
}

/**
  * @brief  Configure an Access Point

  * @param  ssid : SSID string
  * @param  pass : Password string
  * @param  ecn : Encryption type
  * @param  channel : channel number
  * @param  max_conn : Max allowed connections
  * @retval Operation status
  */
WIFI_Status_t WIFI_ConfigureAP(uint8_t *ssid, uint8_t *pass, WIFI_Ecn_t ecn, uint8_t channel, uint8_t max_conn)
{
    lwespr_t api_ret = lwespERR;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_ap_set_config((const char *)ssid, (const char *)pass, channel, (lwesp_ecn_t)ecn, max_conn, 0, NULL, NULL, 1);
    if (api_ret == lwespOK) {
        ret = WIFI_STATUS_OK;
    }
    
    return ret;
}

/**
  * @brief  Handle the background events of the wifi module

  * @retval None
*/
WIFI_Status_t WIFI_HandleAPEvents(WIFI_APSettings_t *setting)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Ping an IP address in the network
  * @param  ipaddr : array of the IP address
  * @retval Operation status
  */
WIFI_Status_t WIFI_Ping(uint8_t *ipaddr, uint16_t count, uint16_t interval_ms, int32_t result[])
{
    /* ICMP implemented by NetX Duo */
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Get IP address from URL using DNS
  * @param  location : Host URL
  * @param  ipaddr : array of the IP address
  * @retval Operation status
  */
WIFI_Status_t WIFI_GetHostAddress(const char *location, uint8_t *ipaddr)
{
    /* DNS implemented by NetX Duo */    
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Configure and start a client connection
  * @param  type : Connection type TCP/UDP
  * @param  name : name of the connection
  * @param  ipaddr : IP address of the remote host
  * @param  port : Remote port
  * @param  local_port : Local port
  * @retval Operation status
  */
WIFI_Status_t WIFI_OpenClientConnection(uint32_t *socket, WIFI_Protocol_t type, const char *name, uint8_t *ipaddr, uint16_t port, uint16_t local_port)
{
    lwesp_netconn_p client;
    lwesp_netconn_type_t protocol;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    char ip_in_string[16];

    *socket = 0;

    if (type == WIFI_TCP_PROTOCOL) {
        protocol = LWESP_NETCONN_TYPE_TCP;
    } else { /* type == WIFI_TCP_PROTOCOL */
        protocol = LWESP_NETCONN_TYPE_UDP;
    }

    client = lwesp_netconn_new(protocol);
    if (client != NULL) {

        sprintf(ip_in_string, "%d.%d.%d.%d", ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]);
        
        api_ret = lwesp_netconn_connect(client, ip_in_string, port);
        if (api_ret == lwespOK) {
            *socket = (uint32_t)client;
            ret = WIFI_STATUS_OK;
        } else {
            (void)lwesp_netconn_delete(client);
        }
    }

    return ret;
}

/**
  * @brief  Close client connection
  * @retval Operation status
  */
WIFI_Status_t WIFI_CloseClientConnection(uint32_t socket)
{
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_netconn_close((lwesp_netconn_p)socket);
    if (api_ret == lwespOK) {
        api_ret = lwesp_netconn_delete((lwesp_netconn_p)socket);
        if (api_ret == lwespOK) {
            ret = WIFI_STATUS_OK;
        }
    }

    return ret;
}

/**
  * @brief  Configure and start a Server
  * @param  type : Connection type TCP/UDP
  * @param  name : name of the connection
  * @param  port : Remote port
  * @retval Operation status
  */
WIFI_Status_t WIFI_StartServer(uint32_t *socket, WIFI_Protocol_t protocol, uint16_t backlog ,const char *name, uint16_t port)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Wait for a client connection to the server
  * @param  socket : socket
  * @retval Operation status
  */
WIFI_Status_t WIFI_WaitServerConnection(uint32_t socket,uint32_t Timeout,uint8_t *RemoteIp,uint16_t *RemotePort)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Close current connection from a client  to the server
  * @retval Operation status
  */
WIFI_Status_t WIFI_CloseServerConnection(uint32_t socket)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Stop a server
  * @param  socket : socket
  * @retval Operation status
  */
WIFI_Status_t WIFI_StopServer(uint32_t socket)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Send Data on a socket
  * @param  pdata : pointer to data to be sent
  * @param  Reqlen : length of data to be sent
  * @param  SentDatalen : (OUT) length actually sent
  * @param  Timeout : Socket write timeout (ms)
  * @retval Operation status
  */
WIFI_Status_t WIFI_SendData(uint32_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *SentDatalen, uint32_t Timeout)
{
    lwespr_t api_ret = lwespERR;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;
    *SentDatalen = 0;

    if (lwesp_netconn_get_type((lwesp_netconn_p)socket) == LWESP_NETCONN_TYPE_TCP) {
        api_ret = lwesp_netconn_write((lwesp_netconn_p)socket, pdata, (size_t)Reqlen);
        if (api_ret == lwespOK) {
            api_ret = lwesp_netconn_flush((lwesp_netconn_p)socket);
            if (api_ret == lwespOK) {
                *SentDatalen = Reqlen;
                ret = WIFI_STATUS_OK;
            }
        }
    } else if (lwesp_netconn_get_type((lwesp_netconn_p)socket) == LWESP_NETCONN_TYPE_UDP) {
        api_ret = lwesp_netconn_send((lwesp_netconn_p)socket, pdata, (size_t)Reqlen);
        if (api_ret == lwespOK) {
            *SentDatalen = Reqlen;
            ret = WIFI_STATUS_OK;
        }
    }

    return ret;    
}

/**
  * @brief  Send Data on a socket
  * @param  pdata : pointer to data to be sent
  * @param  Reqlen : length of data to be sent
  * @param  SentDatalen : (OUT) length actually sent
  * @param  Timeout : Socket write timeout (ms)
  * @param  ipaddr : (IN) 4-byte array containing the IP address of the remote host
  * @param  port : (IN) port number of the remote host
  * @retval Operation status
  */
WIFI_Status_t WIFI_SendDataTo(uint8_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *SentDatalen, uint32_t Timeout, uint8_t *ipaddr, uint16_t port)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Receive Data from a socket
  * @param  pdata : pointer to Rx buffer
  * @param  Reqlen : maximum length of the data to be received
  * @param  RcvDatalen : (OUT) length of the data actually received
  * @param  Timeout : Socket read timeout (ms)
  * @retval Operation status
  */
WIFI_Status_t WIFI_ReceiveData(uint32_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *RcvDatalen, uint32_t Timeout)
{
    lwesp_pbuf_p pbuf;
    lwespr_t api_ret;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    (void)Timeout;
    *RcvDatalen = 0;

    lwesp_netconn_set_receive_timeout((lwesp_netconn_p)socket, LWESP_NETCONN_RECEIVE_NO_WAIT);

    api_ret = lwesp_netconn_receive((lwesp_netconn_p)socket, &pbuf);
    if (api_ret == lwespCLOSED) {
        printf("Connection closed by remote side...\r\n");
    } else if (api_ret == lwespTIMEOUT) {
        ret = WIFI_STATUS_OK;
    }

    if (api_ret == lwespOK && pbuf != NULL) {
        *RcvDatalen = lwesp_pbuf_length(pbuf, 1);
        memcpy(pdata, lwesp_pbuf_data(pbuf), *RcvDatalen);
        lwesp_pbuf_free(pbuf);    /* Free the memory after usage */
        ret = WIFI_STATUS_OK;
    }

    return ret;      
}

/**
  * @brief  Receive Data from a socket
  * @param  pdata : pointer to Rx buffer
  * @param  Reqlen : maximum length of the data to be received
  * @param  RcvDatalen : (OUT) length of the data actually received
  * @param  Timeout : Socket read timeout (ms)
  * @param  ipaddr : (OUT) 4-byte array containing the IP address of the remote host
  * @param  port : (OUT) port number of the remote host
  * @retval Operation status
  */
WIFI_Status_t WIFI_ReceiveDataFrom(uint8_t socket, uint8_t *pdata, uint16_t Reqlen, uint16_t *RcvDatalen, uint32_t Timeout, uint8_t *ipaddr, uint16_t *port)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Customize module data
  * @param  name : MFC name
  * @param  Mac :  Mac Address
  * @retval Operation status
  */
WIFI_Status_t WIFI_SetOEMProperties(const char *name, uint8_t *Mac)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Reset the WIFI module
  * @retval Operation status
  */
WIFI_Status_t WIFI_ResetModule(void)
{
    lwespr_t api_ret = lwespERR;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_reset(NULL, NULL, 1); 
    if (api_ret == lwespOK) {
        ret = WIFI_STATUS_OK;
    }

    return ret;
}

/**
  * @brief  Restore module default configuration
  * @retval Operation status
  */
WIFI_Status_t WIFI_SetModuleDefault(void)
{
    lwespr_t api_ret = lwespERR;
    WIFI_Status_t ret = WIFI_STATUS_ERROR;

    api_ret = lwesp_restore(NULL, NULL, 1); 
    if (api_ret == lwespOK) {
        ret = WIFI_STATUS_OK;
    }

    return ret;
}


/**
  * @brief  Update module firmware
  * @param  location : Binary Location IP address
  * @retval Operation status
  */
WIFI_Status_t WIFI_ModuleFirmwareUpdate(const char *location)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Return Module firmware revision
  * @param  rev : revision string
  * @retval Operation status
  */
WIFI_Status_t WIFI_GetModuleFwRevision(char *rev)
{
    lwesp_sw_version_t version;
    lwesp_get_current_at_fw_version(&version);

    sprintf(rev, "ESP-AT %d.%d.%d", version.major, version.minor, version.patch);
    
    return WIFI_STATUS_OK;
}

/**
  * @brief  Return Module ID
  * @param  Info : Module ID string
  * @retval Operation status
  */
WIFI_Status_t WIFI_GetModuleID(char *Id)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Return Module Name
  * @param  Info : Module Name string
  * @retval Operation status
  */
WIFI_Status_t WIFI_GetModuleName(char *ModuleName)
{
    if (lwesp_device_is_esp8266() == 1) {
        strcpy(ModuleName, "ESP8266");
    } else if (lwesp_device_is_esp32() == 1) {
        strcpy(ModuleName, "ESP32");
    } else if (lwesp_device_is_esp32_c3() == 1) {
        strcpy(ModuleName, "ESP32-C3");
    } else {
        strcpy(ModuleName, "UNKNOWN");
    }

    return WIFI_STATUS_OK;
}
