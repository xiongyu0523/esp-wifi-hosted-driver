#include <stdint.h>

#include "nx_api.h"
#include "wifi.h"

typedef union {
    uint32_t value;
    uint8_t  byte[4];
} ipv4addr;

#ifndef RETRY_TIMES
#define RETRY_TIMES 3
#endif

#define TEST_AP_MODE                    1

#if (TEST_AP_MODE == 1)
#define WIFI_CHANNEL 13
#endif


void sample_network_configure(const CHAR *ssid_ptr, const CHAR *pwd_ptr, NX_IP *ip_ptr, ULONG *dns_address)
{
    uint32_t retry_connect = 0;

    uint8_t mac_address[6];
    char fw_version[20];
    ipv4addr ip_address;
    ipv4addr network_mask;
    ipv4addr gateway_address;
    ipv4addr dns_address_first;
    ipv4addr dns_address_second;

    UINT    status;

    if (WIFI_Init() == WIFI_STATUS_OK) {

        printf("WIFI Initialize Succeed.\r\n");

        WIFI_GetModuleFwRevision(fw_version);
        printf("> Fw Version: %s\r\n", fw_version);   

        if(WIFI_GetMAC_Address(mac_address) == WIFI_STATUS_OK) {       
            printf("WIFI MAC Address: %X:%X:%X:%X:%X:%X\r\n", 
                    mac_address[0], mac_address[1], mac_address[2], mac_address[3], mac_address[4], mac_address[5]); 
        } else {
            printf("!!!ERROR: WIFI Get MAC Address Failed.\r\n");
            return;
        }

#if (TEST_AP_MODE == 1)
        if (WIFI_ConfigureAP("mytestap", "12345678", WIFI_ECN_WPA2_PSK, WIFI_CHANNEL, 1) == WIFI_STATUS_OK) {
            printf("WIFI AP is Up.\r\n");
        } else {
            printf("!!!ERROR: WIFI AP Configuration Failed.\r\n");
            return;
        }
#endif
   
        while ((retry_connect++) < RETRY_TIMES) {

            printf("WIFI connect try %d times\r\n", retry_connect);

            if (WIFI_Connect(ssid_ptr, pwd_ptr, WIFI_ECN_WPA2_PSK) == WIFI_STATUS_OK) {

                printf("WIFI Connected to AP\r\n");

                if (WIFI_GetIP_Address((uint8_t *)&ip_address.value) == WIFI_STATUS_OK) {

                    printf("> WIFI IP Address: %d.%d.%d.%d\r\n", 
                            ip_address.byte[0], ip_address.byte[1], ip_address.byte[2], ip_address.byte[3]);
                } else {
                    printf("!!!ERROR: WIFI Get IP Address Failed.\r\n");
                    WIFI_Disconnect();
                    return;
                }

                if (WIFI_GetIP_Mask((uint8_t *)&network_mask.value) == WIFI_STATUS_OK) {
                    printf("> WIFI Net Mask: %d.%d.%d.%d\r\n", 
                            network_mask.byte[0], network_mask.byte[1], network_mask.byte[2], network_mask.byte[3]); 
                } else {
                    printf("!!!ERROR: WIFI Get Net Mask Failed.\r\n");
                    WIFI_Disconnect();
                    return;
                }

                if (WIFI_GetGateway_Address((uint8_t *)&gateway_address.value) == WIFI_STATUS_OK) {
                    printf("> WIFI Gateway Address: %d.%d.%d.%d\r\n", 
                            gateway_address.byte[0], gateway_address.byte[1], gateway_address.byte[2], gateway_address.byte[3]); 
                } else {
                    printf("!!!ERROR: WIFI Get Gateway Address Failed.\r\n");
                    WIFI_Disconnect();
                    return;
                }
        
                if (WIFI_GetDNS_Address((uint8_t *)&dns_address_first.value, (uint8_t *)&dns_address_second.value) == WIFI_STATUS_OK) {

                    printf("> WIFI DNS1 Address: %d.%d.%d.%d\r\n",  
                            dns_address_first.byte[0], dns_address_first.byte[1], dns_address_first.byte[2], dns_address_first.byte[3]); 
                    
                    printf("> WIFI DNS2 Address: %d.%d.%d.%d\r\n",
                            dns_address_second.byte[0], dns_address_second.byte[1], dns_address_second.byte[2], dns_address_second.byte[3]);          
                } else {
                    printf("!!!ERROR: WIFI Get DNS Address Failed.\r\n");
                    WIFI_Disconnect();
                    return;
                }

                break;
            }
        }

        if (retry_connect > RETRY_TIMES)
        {
            printf("!!!ERROR: WIFI can't connected after retry.\r\n");
            return;
        }
    }
    else
    {
        printf("!!!ERROR: WIFI Initialize Failed.\r\n"); 
        return;
    }

    NX_CHANGE_ULONG_ENDIAN(ip_address.value);
    NX_CHANGE_ULONG_ENDIAN(network_mask.value);
    NX_CHANGE_ULONG_ENDIAN(gateway_address.value);
    NX_CHANGE_ULONG_ENDIAN(dns_address_first.value);

    if (dns_address != NULL) {
        *dns_address = dns_address_first.value;
    }

    status = nx_ip_address_set(ip_ptr, ip_address.value, network_mask.value);
    if (status) {
        printf("IP ADDRESS SET FAIL.\r\n");
        return;
    }

    status = nx_ip_gateway_address_set(ip_ptr, gateway_address.value);
    if (status) {
        printf("IP GATEWAY ADDRESS SET FAIL.\r\n");
        return;
    }
}