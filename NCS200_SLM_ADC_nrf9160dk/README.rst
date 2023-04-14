ADC function on serial LTE modem example
##########################

.. contents::
   :local:
   :depth: 2

nRF9160 serial LTE modem example은 nRF9160의 GPIO, TWI를 AT command를 이용하여 control할 수 있도록 reference가 구성되어 있습니다.여기에 추가적으로 nRF9160의  ADC 기능을 AT command로 사용할 수 있도록 구성한 샘플 코드 입니다.

 nRF9160의  ADC는 아래 devzone 링크를 참조 하였습니다.
(https://devzone.nordicsemi.com/f/nordic-q-a/65766/example-code-for-a-d)

Requirements
************

Software Delopment Kit : NCS 2.0.0

Development Kit : nRF52 DK

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

Advertising interval을 변경하기 위해서는 아래 AT command를 순차적으로 입력합니다.

a. AT+ADVSTOP\\r\\n
#. AT+ADVINT=xxxx\\r\\n : set advertising interval

위 AT command 입력 후 변경된 Advertising interval로 Advertising이 다시 시작됩니다.

Tx power를 변경하기 위해서는 아래 AT command를 순차적으로 입력합니다.

a. AT+ADVSTOP\\r\\n
#. AT+TXPWR=+x\\r\\n or AT+TXPWR=-xx\\r\\n : set tx power

위 AT command 입력 후 변경된 Tx power로 Advertising이 다시 시작됩니다.

Device name을 변경하기 위해서는 아래 AT command를 순차적으로 입력합니다.

a. AT+ADVSTOP\\r\\n
#. AT+NAME=xxxxxx\\r\\n : set BLE device name

위 AT command 입력 후 변경된 Device name으로 Advertising이 다시 시작됩니다.

Testing
********************

nRF52 DK에 download한 후 아래 순서로 진행합니다.

1. nRF52 DK를 USB로 PC와 연결 후 power on 합니다.
#. PC에서 테라텀과 같은 터미널 프로그램 실행합니다.
#. 터미널 입력창에서 AT command를 입력하여 테스트를 진행합니다. 

