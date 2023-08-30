#ifndef __GYRO_H__
#define __GYRO_H__

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"


#define GYRO_ADDR 0x6B
#define GYRO_INT_PIN NRF_GPIO_PIN_MAP(0,6)

bool gyro_init(void);
bool gyro_deinit(void);

void gyro_handle_interrupt(void);

float gyro_get_yaw_rate(void);
float gyro_get_pitch_rate(void);
uint64_t gyro_get_sample_count(void);

void gyro_update_biases(void);
bool gyro_stream_enabled(void);
sensor_t* gyro_get_sensor_class(void);

volatile bool gyro_ready(void);
void gyro_reset_biases(void);

#endif