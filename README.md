# Nordic Example code(INSEM Inc)
---
## **NCS200_BLE_AT_command_nrf52dk**

1. **Description**

     이 예제는 PC에서 터미널 프로그램을 사용하여 nRF52 DK의 **BLE 동작(Advertising interval, Tx power 등)을 변경하기 위한 프로젝트** 입니다.

     BLE 동작 변경을 위해 **AT command를 사용**하며 nRF52 DK로부터 AT command에 대한 response를 확인할 수 있으며 BLE 동작 변경은 Central 장치와 connection하지 않은 상태에서 가능하며 **Central 장치와 connection이 되었을 경우 일반적인 BLE 장치로서 동작**합니다.

2. **Reference example**

     NCS(nRF Connect SDK) 2.0.0 BLE peripheral uart


---
## **NCS200_SLM_ADC_nrf9160dk**

1. **Description**

     nRF9160 serial LTE modem example은 nRF9160의 GPIO, TWI를 AT command를 이용하여 control할 수 있도록 reference가 구성되어 있습니다. 여기에 추가적으로 **nRF9160의  ADC 기능을 AT command로 사용**할 수 있도록 구성한 샘플 코드 입니다.

2. **Reference example**

     NCS(nRF Connect SDK) 2.0.0 serial LTE modem


---
## **NCS200_SLM_HTTPS_ip_connection_nrf9160dk**

1. **Description**

     nRF9160 serial LTE modem example을 이용하여 HTTPS server와 연결하려는 경우 hostname을 사용해야 합니다. 
     (https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.0.0/nrf/applications/serial_lte_modem/doc/HTTPC_AT_commands.html)
     
     **hostname없이 IP address로 연결할 경우 error가 발생**합니다. Hostname이 아닌 IP address만을 사용해야 하는 경우 **serial LTE modem에서의 수정** 부분에 대한 샘플코드 입니다.

2. **Reference example**

     NCS(nRF Connect SDK) 2.0.0 serial LTE modem


---
## **NCS200_SLM_RTC_nrf9160dk**

1. **Description**

     이 project는 nRF9160 serial LTE modem example에 **RTC 기능**을 추가한 샘플 코드 입니다.

2. **Reference example**

     NCS(nRF Connect SDK) 2.0.0 serial LTE modem


---
## **NCS230_BLE_Throughput_Peripheral_Uart_nrf52840dk**

1. **Description**

     이 project는 **Bluetooth peripheral uart example을 기반으로 central장치와의 BLE throughput을 향상**시키기 위해서 구성한 샘플 코드 입니다.

2. **Reference example**

     NCS(nRF Connect SDK) 2.3.0 BLE peripheral uart


---
## **NCS230_I2C_SPI_bitbang_nrf52840dk**

1. **Description**

     이 project는 **Software I2C, SPI를 사용**하기 위한 샘플 코드 입니다.

2. **Reference example**

     NCS(nRF Connect SDK) 2.3.0


---
## **nRF5SDK_BLE_2_uarts_nrf52840dk**

1. **Description**

     이 예제는 **BLE peripheral ble app uart example에서 2개의 uart를 사용**하기 위해 테스트로 구성한 프로젝트 입니다. nRF52840 DK 기준으로 구성하였습니다.
 
     
2. **Reference example**

     nRF5 SDK 17.1.0 ble app uart
     
     
---
## **nRF5SDK_BLE_AT_command_nrf52dk**

1. **Description**

     이 예제는 PC에서 터미널 프로그램을 사용하여 nRF52 DK의 **BLE 동작(Advertising interval, Tx power 등)을 변경하기 위한 프로젝트** 입니다. 
     
     BLE 동작 변경을 위해 **AT command를 사용**하며 nRF52 DK로부터 AT command에 대한 response를 확인할 수 있습니다. BLE 동작 변경은 Central 장치와 connection하지 않은 상태에서 가능하며 **Central 장치와 connection이 되었을 경우 일반적인 BLE 장치로서 동작**합니다. 
     
2. **Reference example**

     nRF5 SDK 17.1.0 ble app uart


---
## **nRF5SDK_BLE_minimize_current_nrf52dk**

1. **Description**

     이 예제는 **BLE peripheral 장치의 current consumption을 최소화** 하기 위한 firwmare 수정 부분에 대해 알려드리는 프로젝트 입니다. 
 
     
2. **Reference example**

     nRF5 SDK 17.1.0 ble app uart



