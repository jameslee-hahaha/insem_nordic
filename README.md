# Nordic Example code(INSEM Inc)
---
## **NCS200_ble_AT_command_nrf52dk**

1. **Description** : BLE AT command sample demonstrates how to use the AT command to change BLE operation. It sends AT command and receive a response between a UART connection. If it's connected with BLE central device, normal NUS service works.

2. **Requirements**
  - DK : nRF52 DK
  - SDK : **NCS 2.0.0**
3. **Avaiable AT commands**     
  - AT?\\r\\n : checks AT command mode
  - AT+VER?\\r\\n : returns AT command version
  - AT+ROLE?\\r\\n : return current BLE mode(peripheral)
  - AT+ADVSTOP\\r\\n : stops BLE advertising
  - AT+ADVSTART\\r\\n : start BLE advertising
  - AT+ADVINT?\\r\\n : returns current advertising interval
  - AT+TXPWR?\\r\\n : returns current tx power
  - AT+NAME?\\r\\n : returns current BLE device name
  - AT+SLEEP\\r\\n : enters system off mode after setup wake up
  - AT+ADVINT=xxxx\\r\\n : set advertising interval
  - AT+TXPWR=+x\\r\\n or AT+TXPWR=-xx\\r\\n : set tx power
  - AT+NAME=xxxxxx\\r\\n : set BLE device name

---
## **nRF5SDK_ble_AT_command_nrf52dk**

1. **Description** : BLE AT command sample demonstrates how to use the AT command to change BLE operation. It sends AT command and receive a response between a UART connection. If it's connected with BLE central device, normal NUS service works.

2. **Requirements**
  - DK : nRF52 DK
  - SDK : **nRF5 SDK 17.1.0**
3. **Avaiable AT commands**         
  - AT?\\r\\n : checks AT command mode
  - AT+VER?\\r\\n : returns AT command version
  - AT+ROLE?\\r\\n : return current BLE mode(peripheral)
  - AT+ADVSTOP\\r\\n : stops BLE advertising
  - AT+ADVSTART\\r\\n : start BLE advertising
  - AT+ADVINT?\\r\\n : returns current advertising interval
  - AT+TXPWR?\\r\\n : returns current tx power
  - AT+NAME?\\r\\n : returns current BLE device name
  - AT+SLEEP\\r\\n : enters system off mode after setup wake up
  - AT+ADVINT=xxxx\\r\\n : set advertising interval
  - AT+TXPWR=+x\\r\\n or AT+TXPWR=-xx\\r\\n : set tx power
  - AT+NAME=xxxxxx\\r\\n : set BLE device name   

---
## **ble_minimize_current_solution**

1. **Description** : This example is started from ble_app_uart example. The target of this example is what is should consider to reduce current consumption. So if you have interests about this kinds of things, Please refer to this example.

2. **Requirements**
  - DK : nRF52 DK
  - SDK : **nRF5 SDK 17.1.0**
3. **Notice** : if you check out comments below, you can find out what is changed to minimize current consumption.         
  - STEP_1 Log Disable
  - STEP_2 idle Sleep
  - STEP_3 DCDC Mode enable
  - STEP_4 Advertising interval
  - STEP_5 Non Connectable Advertising(**if your device doesn't need to connect any other BLE central device, use 'ble_advertising_non_connectable.c'**)
  - STEP_6 Connection Parameter (Connection interval, Slave Latency, Supervisor timeout)
  - STEP_7 TX Power
  - STEP_8 Advertising Stop / Start
  - STEP_9 System Off Sleep
  - STEP_10 UART Disable 
