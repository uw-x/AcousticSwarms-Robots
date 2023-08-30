#ifndef IR_H
#define IR_H

#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"

#define IR_ENABLE_PIN NRF_GPIO_PIN_MAP(0, 27)
#define SENSOR_LEFT NRF_GPIO_PIN_MAP(0, 11)
#define SENSOR_RIGHT NRF_GPIO_PIN_MAP(0, 23)


typedef enum {
    IR_SENSORS_NONE   = 0, //no sensors are set.
    IR_SENSORS_LEFT  = 1, //only right sensor is set.
    IR_SENSORS_RIGHT  = 2, //only left sensor is set.
    IR_SENSORS_BOTH   = 3, //both sensors are set.
} ir_sensors_status_t;

typedef enum{ 
  IR_NONE = 0,
  IR_MODE_COUNTER,
  IR_MODE_EDGE_DETECTION,
  IR_DETECT_LEFT,
  IR_DETECT_RIGHT,
  IR_DETECT_NONE,
  IR_DETECT_BOTH,
  IR_MODE_CHARGING_STATION_ENTRY,
} ir_mode;

/**
 * @brief Function for initializing the sensor interrupts.
 *
 * @return true if interrupts successfully initialize.
 */
bool ir_init(void);

/**
 * @brief Function for enabling the sensor interrupts.
 *
 */
void ir_enable(bool);

/**
 * @brief Function for disabling the sensor interrupts.
 *
 */

bool ir_enabled();

void ir_update_status(void);

void ir_disable(void);

void ir_set_mode(ir_mode _mode);

ir_mode ir_get_mode(void);

bool ir_target_depth_reached(void);

void ir_set_target_depth(uint8_t depth);

void ir_increment_marker_count(void);

volatile ir_sensors_status_t ir_get_previous_status(void);

/**
 * @brief Function for finding the current status of the sensors.
 *
 * @return IR_SENSORS_BOTH if both sensors are set. 
           IR_SENSORS_LEFT if only left sensor is set. 
           IR_SENSORS_RIGHT if only right sensor is set. 
           IR_SENSORS_NONE if no sensors are set.
 */
volatile ir_sensors_status_t ir_get_current_status(void);

void ir_enable_stream(bool);
bool ir_streaming_enabled(void);

#endif