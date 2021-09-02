# ESP WiFi Offload driver sample for Azure IoT

This sample code implements ESP offload driver for Azure RTOS NetX Duo and provide a Azure IoT example. It leverages [LwESP](https://github.com/MaJerle/lwesp) project to get WiFi and network support, and integrates with NetX Duo 6.1.8 for offload driver. 

## Hardware connection

The demo is verified on [B-L4S5I-IOT01A](https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html) and [ESP32-C3-DevKitM-1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html) hardware.  

| Signal name | B-L4S5I-IOT01A | ESP32-C3-DevKitM-1 |
| ---- |  ----  | ---- |
| 3V3  | CN2-4 | J1-21  |
| GND | CN2-6   | J1-1   | 
| TX  | CN3-2 (UART4-TX) | J3-9 (UART1-RX) | 
| RX  | CN3-1 (UART4-RX) | J3-8 (URAT1-TX) | 
| RESET | CN3-3 (GPIOD14) | CN1-7 (EN)

## Prequisite 

ESP32-C3 must be programmed with ESP-AT firmware. Refer to this [link](https://download.espressif.com/esp_at/firmware/ESP32C3/ESP32-C3-MINI-1_AT_Bin_V2.2.0.0.zip) for pre-built binary and document. 


## License

This repository inherit Azure RTOS license from Microsoft. See [LICENSE.txt](./LICENSE.txt) and [LICENSED-HARDWARE.txt](./LICENSED-HARDWARE.txt).
