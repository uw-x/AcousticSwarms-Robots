#ifndef _I2C_H_
#define _I2C_H_

#include "i2c_config.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"

#define MAX_I2C_PERIPHERALS 4

#define TWI0_I2C_SCL_PIN NRF_GPIO_PIN_MAP(0, 8)
#define TWI0_I2C_SDA_PIN NRF_GPIO_PIN_MAP(0, 4)

//#define TWI1_I2C_SCL_PIN NRF_GPIO_PIN_MAP(1, 8)
//#define TWI1_I2C_SDA_PIN NRF_GPIO_PIN_MAP(0, 6)

#define TWI1_I2C_SCL_PIN NRF_GPIO_PIN_MAP(0, 19)
#define TWI1_I2C_SDA_PIN NRF_GPIO_PIN_MAP(1, 8)

#define I2C_FREQ   NRF_TWI_FREQ_250K
#define I2C_IRQ_PRI APP_IRQ_PRIORITY_LOW
#define SCL_PIN_INIT_CONF     ( (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
                              | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos) \
                              | (GPIO_PIN_CNF_PULL_Pullup    << GPIO_PIN_CNF_PULL_Pos)  \
                              | (GPIO_PIN_CNF_INPUT_Connect  << GPIO_PIN_CNF_INPUT_Pos) \
                              | (GPIO_PIN_CNF_DIR_Input      << GPIO_PIN_CNF_DIR_Pos))
#define SDA_PIN_INIT_CONF        SCL_PIN_INIT_CONF

typedef struct {
  uint32_t addr;
  nrf_drv_twi_evt_handler_t evt_handler;
  uint8_t twi_instance_id;
} i2c_peripheral_t;

bool i2c_init(void);
bool i2c_deinit(void);

bool i2c_register_peripheral(i2c_peripheral_t *p);
bool i2c_write(i2c_peripheral_t *p, uint8_t reg_addr, const uint8_t * const data, uint8_t length);
bool i2c_read(i2c_peripheral_t *p, uint8_t reg_addr, uint8_t *data, uint8_t length);

uint16_t i2c_read16(i2c_peripheral_t *p, uint8_t reg_addr);
bool i2c_write16(i2c_peripheral_t *p, uint8_t reg_addr, uint16_t val);


#endif