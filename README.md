## Nordic Example code(INSEM Inc)

---
- **NCS200_ble_AT_command_nrf52dk**

   - Description : This example is based on *ble peripheral uart*. if user want to change some operation regarding BLE, BLE AT command can be used. Like a putty or tera term can be used to sent commands.

   - This example can be tested on environment like below.
  
     DK : nRF52DK
    
     SDK : NCS 2.0.0
        
   - Avaiable AT commands

     AT?\r\n : checks AT command mode
     
     AT+VER?\r\n : returns AT command version
     
     AT+ROLE?\r\n : return current BLE mode(peripheral)
     
     AT+ADVSTOP\r\n : stops BLE advertising
     
     AT+ADVSTART\r\n : start BLE advertising
     
     AT+ADVINT?\r\n : returns current advertising interval
     
     AT+ADVINT=xxxx\r\n : set advertising interval
     
     AT+TXPWR?\r\n : returns current tx power
     
     AT+TXPWR=+x\r\n or AT+TXPWR=-xx\r\n : set tx power
     
     AT+NAME?\r\n : returns current BLE device name
     
     AT+NAME=xxxxxx\r\n : set BLE device name
     
     AT+SLEEP\r\n : enters system off mode after setup wake up
     
---
- **nRF5SDK_ble_AT_command_nrf52dk**

   - Description : This example is based on *ble app uart*. if user want to change some operation regarding BLE, BLE AT command can be used. Like a putty or tera term can be used to sent commands.

   - This example can be tested on environment like below.
  
     DK : nRF52DK
    
     SDK : nRF5 SDK 17.1.0
        
   - Avaiable AT commands

     AT?\r\n : checks AT command mode
     
     AT+VER?\r\n : returns AT command version
     
     AT+ROLE?\r\n : return current BLE mode(peripheral)
     
     AT+ADVSTOP\r\n : stops BLE advertising
     
     AT+ADVSTART\r\n : start BLE advertising
     
     AT+ADVINT?\r\n : returns current advertising interval
     
     AT+ADVINT=xxxx\r\n : set advertising interval
     
     AT+TXPWR?\r\n : returns current tx power
     
     AT+TXPWR=+x\r\n or AT+TXPWR=-xx\r\n : set tx power
     
     AT+NAME?\r\n : returns current BLE device name
     
     AT+NAME=xxxxxx\r\n : set BLE device name
     
     AT+SLEEP\r\n : enters system off mode after setup wake up
     

