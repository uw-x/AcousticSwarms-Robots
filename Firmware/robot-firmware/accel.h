#ifndef _ACCEL_H_
#define _ACCEL_H_

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC34X9_RETCODE_SUCCESS                 (0)
#define MC34X9_RETCODE_ERROR_BUS               (-1)
#define MC34X9_RETCODE_ERROR_NULL_POINTER      (-2)
#define MC34X9_RETCODE_ERROR_STATUS            (-3)
#define MC34X9_RETCODE_ERROR_SETUP             (-4)
#define MC34X9_RETCODE_ERROR_GET_DATA          (-5)
#define MC34X9_RETCODE_ERROR_IDENTIFICATION    (-6)
#define MC34X9_RETCODE_ERROR_NO_DATA           (-7)
#define MC34X9_RETCODE_ERROR_WRONG_ARGUMENT    (-8)
#define MC34X9_FIFO_DEPTH                        32
#define MC34X9_REG_MAP_SIZE                      64

/******************************************************************************
 *** CONSTANT / DEFINE
 *****************************************************************************/
#define MC34X9_INTR_C_IPP_MODE_OPEN_DRAIN    (0x00)
#define MC34X9_INTR_C_IPP_MODE_PUSH_PULL     (0x01)

#define MC34X9_INTR_C_IAH_ACTIVE_LOW         (0x00)
#define MC34X9_INTR_C_IAH_ACTIVE_HIGH        (0x01)

#define MC34X9_AUTO_CLR_DISABLE              (0x00)
#define MC34X9_AUTO_CLR_ENABLE               (0x01)
/******************************************************************************
 *** Register Map
 *****************************************************************************/
#define MC34X9_REG_DEV_STAT         (0x05)
#define MC34X9_REG_INTR_CTRL        (0x06)
#define MC34X9_REG_MODE             (0x07)
#define MC34X9_REG_SR               (0x08)
#define MC34X9_REG_MOTION_CTRL      (0x09)
#define MC34X9_REG_FIFO_STAT        (0x0A)
#define MC34X9_REG_FIFO_RD_P        (0x0B)
#define MC34X9_REG_FIFO_WR_P        (0x0C)
#define MC34X9_REG_XOUT_LSB         (0x0D)
#define MC34X9_REG_XOUT_MSB         (0x0E)
#define MC34X9_REG_YOUT_LSB         (0x0F)
#define MC34X9_REG_YOUT_MSB         (0x10)
#define MC34X9_REG_ZOUT_LSB         (0x11)
#define MC34X9_REG_ZOUT_MSB         (0x12)
#define MC34X9_REG_STATUS           (0x13)
#define MC34X9_REG_INTR_STAT        (0x14)
#define MC34X9_REG_PROD             (0x18)
#define MC34X9_REG_RANGE_C          (0x20)
#define MC34X9_REG_XOFFL            (0x21)
#define MC34X9_REG_XOFFH            (0x22)
#define MC34X9_REG_YOFFL            (0x23)
#define MC34X9_REG_YOFFH            (0x24)
#define MC34X9_REG_ZOFFL            (0x25)
#define MC34X9_REG_ZOFFH            (0x26)
#define MC34X9_REG_XGAIN            (0x27)
#define MC34X9_REG_YGAIN            (0x28)
#define MC34X9_REG_ZGAIN            (0x29)
#define MC34X9_REG_FIFO_CTRL        (0x2D)
#define MC34X9_REG_FIFO_TH          (0x2E)
#define MC34X9_REG_FIFO_INTR        (0x2F)
#define MC34X9_REG_FIFO_CTRL_SR2    (0x30)
#define MC34X9_REG_COMM_CTRL        (0x31)
#define MC34X9_REG_GPIO_CTRL        (0x33)
#define MC34X9_REG_TF_THRESH_LSB    (0x40)
#define MC34X9_REG_TF_THRESH_MSB    (0x41)
#define MC34X9_REG_TF_DB            (0x42)
#define MC34X9_REG_AM_THRESH_LSB    (0x43)
#define MC34X9_REG_AM_THRESH_MSB    (0x44)
#define MC34X9_REG_AM_DB            (0x45)
#define MC34X9_REG_SHK_THRESH_LSB   (0x46)
#define MC34X9_REG_SHK_THRESH_MSB   (0x47)
#define MC34X9_REG_PK_P2P_DUR_THRESH_LSB    (0x48)
#define MC34X9_REG_PK_P2P_DUR_THRESH_MSB    (0x49)
#define MC34X9_REG_TIMER_CTRL       (0x4A)
#define MC34X9_REG_RD_CNT           (0x4B)

#define MC34X9_NULL_ADDR            (0)

#define MC34X9_CHIP_ID (0xA4)

#define MC34X9_SLAVE_ADDR            (0x4c)

#include "stdint.h"
#include "stdbool.h"
#include "sensors.h"


#define ACCEL_INT_PIN NRF_GPIO_PIN_MAP(0, 29)

#define ACCEL_AM_INT_BIT 2
#define ACCEL_ACQ_INT_BIT 7

typedef enum{
  ACCEL_RANGE_2G = 0,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_12G,
  ACCEL_RANGE_16G,
} accel_range_t;

typedef enum{
  COLLISIONS_DETECT_OFF = 0,
  COLLISIONS_DETECT_MOVING,
  COLLISIONS_DETECT_STATIONARY,
} accel_collision_detection_t;

typedef enum{
  ACCEL_STATE_STANDBY = 0,
  ACCEL_STATE_WAKE,
}accel_state_t;


bool accel_init(void);
bool accel_deinit(void);

bool accel_test(void);

bool accel_stream_enabled(void);
sensor_t* accel_get_sensor_class(void);

void accel_handle_interrupt(void);

float accel_get_roll(void);
float accel_get_pitch(void);

void accel_get_xyz(float* arr);
volatile bool accel_ready(void);
void accel_update_biases(void);
void accel_reset_biases(void);

void accel_disable_anymotion(void);
void accel_enable_anymotion(uint16_t threshold);

void accel_enable_collision_detection(bool moving);
void accel_disable_collision_detection(void);

accel_collision_detection_t accel_get_collision_detection_state(void);

#endif