# ESP WiFi Hosted driver sample for Azure IoT

This sample code implements a ESP32 based network driver for Azure RTOS NetX Duo and provide a Azure IoT example. We can it hosted because it leverages [esp-hosted](https://github.com/espressif/esp-hosted) project to get 802.3 adapter support in ESP32 SoC, and integrates the host api with NetX Duo 6.1.8 and general WiFi control APIs.

## Hardware connection

The demo is verified on [B-L4S5I-IOT01A](https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html) and [ESP32-C3-DevKitM-1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/hw-reference/esp32c3/user-guide-devkitm-1.html) or [ESP32-S2-Saola-1](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s2/hw-reference/esp32s2/user-guide-saola-1-v1.2.html) hardware. 

| Signal name | B-L4S5I-IOT01A | ESP32-C3-DevKitM-1 | ESP32-S2-Saola-1 |
| ---- |  ----  | ---- | ---- |
| 3V3  | CN2-2 |  | J2-1  |
| GND | CN2-7 |   | J2-21  |  
| MOSI  | CN1-4 (PA7)  | | J2-13 (IO11)  |
| MISO  | CN1-5 (PA6)  | | J2-15 (IO13)  |
| SCK | CN1-6 (PA5) | | J2-14 (IO12)  |
| CS | CN1-3 (PA2)  | | J2-12 (IO10)  |
| Reset | CN3-5 (PA3)  | | J3-2 (CHIP-PU)  |
| Handshake | CN3-3 (PD14) | | J2-4 (IO2)  |
| Data Ready | CN3-3 (PB0) | | J2-6 (IO4)  |
 
## Prequisite 

ESP32-C3 or ESP32-S2 must be programmed with adapter firmware in [esp-hosted](https://github.com/espressif/esp-hosted/tree/master/esp/esp_driver/network_adapter) project.


## License

This repository inherit Azure RTOS license from Microsoft. See [LICENSE.txt](./LICENSE.txt) and [LICENSED-HARDWARE.txt](./LICENSED-HARDWARE.txt).
