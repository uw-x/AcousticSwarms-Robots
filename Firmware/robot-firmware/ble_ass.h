#ifndef _BLE_ASS_H_
#define _BLE_ASS_H_

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

// Acoustic Streaming Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define AUDIO_STREAMING_SERVICE_UUID_BASE {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define AUDIO_STREAMING_SERVICE_UUID               0x1500
#define AUDIO_STREAMING_VALUE_CHAR_UUID            0x1501

#define AUDIO_STREAMING_SERVICE_NUM_CHARS          4

#define ASS_CONTROL_CHAR_UUID                 0x1503
#define ASS_MIC_CHAR_UUID                  0x1504
#define ASS_SPK_CHAR_UUID                  0x1505
#define ASS_STATUS_CHAR_UUID                 0x1506

// CONTROL
#define ASS_CONTROL_ENABLE_MASTER           0x6D
#define ASS_CONTROL_ENABLE_SLAVE            0x73
#define ASS_CONTROL_TS_DISABLE              0x74
#define ASS_CONTROL_MIC_STREAM_STOP         0xEF
#define ASS_CONTROL_STOP_CLICK              0x10
#define ASS_CONTROL_START_CLICK             0x11
#define ASS_CONTROL_MIC_UPDATE_PARAMS       0x12
#define ASS_CONTROL_MIC_MODE_SET_STREAM     0x13
#define ASS_CONTROL_MIC_MODE_SET_RECORD     0x14
#define ASS_CONTROL_RECORDING_SEND          0x15

// STATUS
#define ASS_STATUS_RECORDING_FINISHED 0x01


typedef enum{
  ASS_CONTROL_CHAR_HANDLE = 0,
  ASS_MIC_CHAR_HANDLE,
  ASS_SPK_CHAR_HANDLE,
  ASS_STATUS_CHAR_HANDLE,
} ble_ass_char_container_idx;

uint32_t ble_ass_init();

void ble_ass_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


#endif