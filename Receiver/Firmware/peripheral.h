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

// Acoustic Streaming Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define AUDIO_STREAMING_SERVICE_UUID_BASE {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define AUDIO_STREAMING_SERVICE_UUID               0x1500
#define AUDIO_STREAMING_VALUE_CHAR_UUID            0x1501

#define AUDIO_STREAMING_SERVICE_NUM_CHARS          3

#define ASS_CONTROL_CHAR_UUID                 0x1503
#define ASS_MIC_CHAR_UUID                  0x1504
#define ASS_SPK_CHAR_UUID                  0x1505
#define ASS_STATUS_CHAR_UUID               0x1506

#define ASS_CONTROL_ENABLE_MASTER           0x6D
#define ASS_CONTROL_ENABLE_SLAVE            0x73
#define ASS_CONTROL_MIC_STREAM_STOP         0xEF
#define ASS_CONTROL_STOP_CLICK              0x10
#define ASS_CONTROL_START_CLICK             0x11
#define ASS_STATUS_RECORDING_FINISHED       0x01


#include "ble_cus.h"

// Sensor Streaming Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define SENSOR_STREAMING_SERVICE_UUID_BASE {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define SENSOR_STREAMING_SERVICE_UUID               0x1600
#define SENSOR_STREAMING_VALUE_CHAR_UUID            0x1601

#define SENSOR_STREAMING_SERVICE_NUM_CHARS          2
#define NUM_SENSORS       2 // Change this when more sensors are added

#define SSS_CONTROL_CHAR_UUID                0x1602
#define SSS_SENSOR_DATA_UUID                 0x1603

#define SSS_SENSOR_PARAMS_CONFIGURE          0xFD
#define SSS_STREAM_CONFIGURE                 0xFE