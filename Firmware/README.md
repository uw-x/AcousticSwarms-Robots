# Firmware
This subfolder contains the source code for the firmware we upload on the robots. The firmware is compiled and uploaded using [Segger Embedded Studio v7.22](https://www.segger.com/downloads/embedded-studio/) and the [nRF5 SDK v17.0.1](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download#infotabs). To upload the firmware, we use a Segger J-Link EDU connected to a robot over the SWD interface exposed via the on-board microUSB port.

## What's included
This folder contains the following directories:

**robot-firmware**: The firmware source code.

**SDK**: Base directory where the SDK should be placed. Please download and extract the SDK into this directory before opening compiling the firmware.

## Opening, compiling and flashing
Open the project ```robot-firmware/pca10056/s140/ses/ble_app_template_pca10056_s140.emproject``` in Segger Embedded Studio. After extracting the SDK into the right directory, the firmware can be compiled using ```Build > Build Solution```. Make sure that the active Build Configuration is set to Release. Once built, connect the robot to the J-Link and flash it using ```Target > Download ble_app_template_pca10056_s140```.
