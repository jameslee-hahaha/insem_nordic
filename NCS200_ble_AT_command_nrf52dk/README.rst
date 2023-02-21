BLE AT command example
##########################

.. contents::
   :local:
   :depth: 2

BLE AT command sample demonstrates how to use the AT command to change BLE operation.It sends AT command and receive a response between a UART connection. If it's connected with BLE central device, normal NUS service works.

Requirements
************

Software Delopment Kit : **NCS 2.0.0**

Development Kit : **nRF52 DK**

Avaiable AT commands
************

1. AT?\\r\\n : checks AT command mode
#. AT+VER?\\r\\n : returns AT command version
#. AT+ROLE?\\r\\n : return current BLE mode(peripheral)
#. AT+ADVSTOP\\r\\n : stops BLE advertising
#. AT+ADVSTART\\r\\n : start BLE advertising
#. AT+ADVINT?\\r\\n : returns current advertising interval
#. AT+TXPWR?\\r\\n : returns current tx power
#. AT+NAME?\\r\\n : returns current BLE device name
#. AT+SLEEP\\r\\n : enters system off mode after setup wake up

**To change advertising interval**, Please send AT commands like below.

a. AT+ADVSTOP\\r\\n
#. AT+ADVINT=xxxx\\r\\n : set advertising interval

After this, advertising will be started automatically.

**To change Tx power**, Please send AT commands like below.

a. AT+ADVSTOP\\r\\n
#. AT+TXPWR=+x\\r\\n or AT+TXPWR=-xx\\r\\n : set tx power

After this, advertising will be started automatically.

**To change device name**, Please send AT commands like below.

a. AT+ADVSTOP\\r\\n
#. AT+NAME=xxxxxx\\r\\n : set BLE device name

After this, advertising will be started automatically.

Testing
********************

After programming the sample to your development kit, complete the following steps to test it:

1. Connect the device to the computer to access UART 0.

   On nRF52 DK, UART 0 is forwarded as a COM port (Windows) or ttyACM device (Linux) after you connect the development kit over USB.
#. Connect terminal
#. Type AT commands and Check operations. 

