#ifndef _MOTORS_H_
#define _MOTORS_H_

#include "nrf_gpio.h"


#define MOTORS_M1I1 NRF_GPIO_PIN_MAP(0, 12)
#define MOTORS_M1I2 NRF_GPIO_PIN_MAP(1, 9)
#define MOTORS_M2I1 NRF_GPIO_PIN_MAP(0, 20)
#define MOTORS_M2I2 NRF_GPIO_PIN_MAP(0, 22)

#define MOTORS_DRV  NRF_GPIO_PIN_MAP(0, 21)

typedef struct {
  uint8_t p, n;
  uint16_t target_speed[2];
  uint16_t current_speed[2];
} motor_t;


bool motors_init(void);
bool motors_deinit(void);

void motors_forward(uint16_t speed);
void motors_backward(uint16_t speed);

void motors_rotate_ccw(uint16_t speed);
void motors_rotate_cw(uint16_t speed);

void motors_rotate_ccw_direct(uint16_t speed);
void motors_rotate_cw_direct(uint16_t speed);

void motors_set(int16_t left, int16_t right);

void motors_hard_brake();
void motors_brake();
void motors_loose();
void motors_adjust(float speed, float beta);

void motors_pulse_forward(uint16_t minval, uint16_t maxval, uint16_t period);
void motors_pulse_backward(uint16_t minval, uint16_t maxval, uint16_t period);

void motors_test();

void motors_start(void);
void motors_stop(void);



#endif