## Nordic Example code(INSEM Inc)
---
- **NCS200_ble_AT_command_nrf52dk**

1. **Description** : The AT command for BLE sample demonstrates how to use the AT command to change BLE operation. It sends AT command and receive a response between a UART connection. If it's connected with BLE central device, normal NUS service works.

2. **Requirements**
  - DK : nRF52DK
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
- **nRF5SDK_ble_AT_command_nrf52dk**

1. **Description** : The AT command for BLE sample demonstrates how to use the AT command to change BLE operation. It sends AT command and receive a response between a UART connection. If it's connected with BLE central device, normal NUS service works.

2. **Requirements**
  - DK : nRF52DK
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

