#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <stdint.h>

typedef void (*sensor_config_handler) (const uint8_t * const cfg_data, uint16_t len);
typedef void (*sensor_calibration_method) (void);

#define SENSOR_FG_ID 0
#define SENSOR_ACCEL_ID 1
#define SENSOR_MAG_ID 2
#define SENSOR_GYRO_ID 3

typedef struct{
  bool stream_enabled;
  bool on_state;
  sensor_config_handler cfg_handler;
  sensor_calibration_method calibrate;
  uint8_t *sensor_data;
  uint8_t data_length;
  uint8_t id;
} sensor_t;

#endif