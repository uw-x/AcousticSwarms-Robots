#ifndef _BLE_SSS_H_
#define _BLE_SSS_H_

#include "ble_cus.h"
#include "sensors.h"

// Sensor Streaming Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define SENSOR_STREAMING_SERVICE_UUID_BASE {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define SENSOR_STREAMING_SERVICE_UUID               0x1600
#define SENSOR_STREAMING_VALUE_CHAR_UUID            0x1601

#define SENSOR_STREAMING_SERVICE_NUM_CHARS          2
#define NUM_SENSORS       4 // Change this when more sensors are added

#define SSS_CONTROL_CHAR_UUID                0x1602
#define SSS_SENSOR_DATA_UUID                 0x1603


#define SSS_READ_FUEL_GAUGE 0xF3
#define SSS_ENABLE_EDGE_DETECTION 0xF4
#define SSS_DISABLE_EDGE_DETECTION 0xF5
#define SSS_IR_STATUS_READ 0xF6
#define SSS_ENABLE_IR_STATUS_STREAM 0xF7
#define SSS_DISABLE_IR_STATUS_STREAM 0xF8
#define SSS_ENABLE_COLLISION_DETECTION 0xF9
#define SSS_DISABLE_COLLISION_DETECTION 0xFA
#define SSS_CALIBRATE_ALL 0xFB
#define SSS_STREAM_ENABLE 0xFC
#define SSS_STREAM_DISABLE 0xFD
#define SSS_SENSOR_PARAMS_CONFIGURE 0xFE



typedef enum{
  SSS_CONTROL_CHAR_HANDLE = 0,
  SSS_SENSOR_DATA_HANDLE,
} ble_sss_char_container_idx;

uint32_t ble_sss_init();

void ble_sss_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
bool ble_sss_notify_sensor_data(ble_cus_t * p_cus, const ble_gatts_char_handles_t * const handles, sensor_t *sensor);

#endif
