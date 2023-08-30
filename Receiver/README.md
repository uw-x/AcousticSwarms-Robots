# Receiver
This subfolder contains the source code and firmware for the receiver used to transmit data to and from the robot swarm. We use an off-the-shelf nRF52840DK to communicate with all the robots over BLE, and pass messages to a host computer using a USB CDC ACM interface. The firmware is compiled and uploaded using [Segger Embedded Studio v7.22](https://www.segger.com/downloads/embedded-studio/) and the [nRF5 SDK v17.0.1](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs).

## What's included
This folder contains the following directories:

**Firmware**: The firmware that we flash the nRF52840DK with.

**Host**: Source code that we run on the a host machine to communicate with the receiver over USB.

**SDK**: Base directory where the SDK should be placed. Please download and extract the SDK into this directory before opening compiling the firmware.

## Opening, compiling and flashing
Open the project ```Firmware/pca10056/s140/ses/ble_app_uart_c_pca10056_s140.emproject``` in Segger Embedded Studio. After extracting the SDK into the right directory, the firmware can be compiled using ```Build > Build Solution```. Make sure that the active Build Configuration is set to Release. Once built, connect the receiver to the J-Link and flash it using ```Target > Download ble_app_uart_c_pca10056_s140```.

## Swarm communication
Most of the implementation for communicating with the swarm can be found in ```Host/Controller.py```. It contains code to connect, issue commands, access various BLE services, and many more. Some examples of how these functions were used can be found in our evaluation examples (```evaluate_*.py```). For convenience during evaluation, robots were labeled A-H and their addresses were stored in ```robots.txt```.
