/* Includes ------------------------------------------------------------------*/
#include "commands.h"

#include "wifi.h"

/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/* Public functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the WIFI core
  * @param  None
  * @retval Operation status
  */
WIFI_Status_t WIFI_Init(void)
{
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  This function retrieves the WiFi interface's MAC address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetMAC_Address(uint8_t  *mac)
{
    return wifi_get_mac(WIFI_MODE_STA, (char *)mac) == SUCCESS ? WIFI_STATUS_OK : WIFI_STATUS_ERROR;
}

    
/**
  * @brief  This function retrieves the WiFi interface's IP address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetIP_Address (uint8_t  *ipaddr)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  This function retrieves the WiFi interface's IP mask.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetIP_Mask (uint8_t  *mask)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  This function retrieves the WiFi interface's Gateway address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetGateway_Address (uint8_t  *Gateway_addr)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  This function retrieves the WiFi interface's DNS address.
  * @retval Operation Status.
  */
WIFI_Status_t WIFI_GetDNS_Address (uint8_t  *DNS1addr,uint8_t  *DNS2addr)
{
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Disconnect from a network
  * @param  None
  * @retval Operation status
  */
WIFI_Status_t WIFI_Disconnect(void)
{
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Close client connection
  * @retval Operation status
  */
WIFI_Status_t WIFI_CloseClientConnection(uint32_t socket)
{
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
}

/**
  * @brief  Restore module default configuration
  * @retval Operation status
  */
WIFI_Status_t WIFI_SetModuleDefault(void)
{
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
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
    return WIFI_STATUS_NOT_SUPPORTED;
}
