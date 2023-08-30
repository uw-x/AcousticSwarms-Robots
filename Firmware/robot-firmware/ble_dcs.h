#ifndef _BLE_DCS_H_
#define _BLE_DCS_H_

#include <stdint.h>
#include <stdbool.h>

#include "ble_cus.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"


// This was written by following:
// https://github.com/bjornspockeli/custom_ble_service_example

// Generate UUIDs here:
// https://www.uuidgenerator.net/version4

// Device Control Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define DEVICE_CONTROL_SERVICE_UUID_BASE {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define DEVICE_CONTROL_SERVICE_UUID               0x1400
#define DEVICE_CONTROL_VALUE_CHAR_UUID            0x1401

#define DEVICE_CONTROL_SERVICE_NUM_CHARS          2

#define DCS_COMMAND_CHAR_UUID                 0x1403
#define DCS_STATUS_CHAR_UUID                  0x1404

#define DCS_COMMAND_PING 0x79
#define DCS_COMMAND_SYNC_TASK_CREATE    0x80
#define DCS_COMMAND_ROBOT_MOTION        0x81
#define DCS_COMMAND_ROBOT_POSITION_UPDATE       0x82
#define DCS_COMMAND_SYNC_TASK_CREATE_RECURRING 0x83
#define DCS_COMMAND_XCORR_STREAM_ENABLE 0x84
#define DCS_COMMAND_XCORR_STREAM_DISABLE 0x85
#define DCS_COMMAND_ADD_MILESTONES 0x86
#define DCS_COMMAND_NAV_BEGIN 0x87
#define DCS_COMMAND_ENTER_BASE 0x88
#define DCS_COMMAND_LOAD_PROFILE 0x89
#define DCS_COMMAND_RESET_MILESTONES 0x90

#define DCS_STATUS_UPDATE_XCORR 0
#define DCS_STATUS_MILESTONE_REACHED 1
#define DCS_STATUS_MOTION_CORRECTION 2
#define DCS_STATUS_NEXT_MILESTONE 3
#define DCS_STATUS_NAV_TRANSITION 4
#define DCS_STATUS_NAV_END 5
#define DCS_STATUS_LOG 6
#define DCS_STATUS_CONNECTION_INTERVAL_UPDATE 7
#define DCS_STATUS_CALIBRATION_DONE 8
#define DCS_STATUS_COLLISION_DETECTED 9
#define DCS_IR_STATUS_UPDATED 10
#define DCS_STATUS_BASE_ENTERED 11
#define DCS_STATUS_PING 12


typedef enum{
  DCS_STATUS_CHAR_HANDLE = 0,
  DCS_COMMAND_CHAR_HANDLE,
} ble_dcs_char_container_idx;

uint32_t ble_dcs_init();

void ble_dcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


#endif