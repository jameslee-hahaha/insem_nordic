Minimizing current consumption example of BLE device
##########################

.. contents::
   :local:
   :depth: 2

This example is started from ble_app_uart example. The target of this example is what is should consider to reduce current consumption. So if you have interests about this kinds of things, Please refer to this example. 

Requirements
************

Software Delopment Kit : **nRF5 SDK 17.1.0**

Development Kit : **nRF52 DK**

Notice
************
if you check out comments below, you can find out what is changed to minimize current consumption.

1. STEP_1 Log Disable
#. STEP_2 idle Sleep
#. STEP_3 DCDC Mode enable
#. STEP_4 Advertising interval
#. STEP_5 Non Connectable Advertising(**if your device doesn't need to connect any other BLE central device, use 'ble_advertising_non_connectable.c'**)
#. STEP_6 Connection Parameter (Connection interval, Slave Latency, Supervisor timeout)
#. STEP_7 TX Power
#. STEP_8 Advertising Stop / Start
#. STEP_9 System Off Sleep
#. STEP_10 UART Disable 


