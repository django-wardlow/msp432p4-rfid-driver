# msp432p4-rfid-driver

A C driver for RFID reading modules for the msp432p4 based on the [MFRC522 Arduino library](https://github.com/miguelbalboa/rfid/tree/master).\
It is currently only tested on an msp432p4111 but should be compatible with all msp432p4 series chips

## Requirements

msp432 driverlib must be installed for the driver to work.\
The Code Composer Studio project is set up to use the driverlib files form an [msp432 simplelink](https://www.ti.com/tool/SIMPLELINK-MSP432-SDK) instillation

## Usage

the driver assumes that the RFID module is connected as follows:\
P3.2 -> SDA\
P1.5 -> SCK\
P1.6 -> MOSI\
P1.7 -> MISO

main.c provides a minimal example that detects a card, reads its uid, and prints the uid out as a number.\
Most of the other features of the library should work but are untested.\
Additionally, some (presumably) unnecessary debugging functions were removed when porting from C++ to C
