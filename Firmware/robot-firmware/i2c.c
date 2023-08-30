#include "i2c.h"
#include "nrf_drv_twi.h"
#include "string.h"
#include "nrf_log.h"


static const nrf_drv_twi_t m_twi[2] = {NRF_DRV_TWI_INSTANCE(0), NRF_DRV_TWI_INSTANCE(1)};
static i2c_peripheral_t* i2c_peripherals[MAX_I2C_PERIPHERALS];

static uint8_t n_peripherals = 0;
static volatile bool m_xfer_done = true;
static uint8_t m_buffer[255];

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){
    switch (p_event->type){
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.address == NRF_DRV_TWI_XFER_RX){
                //data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
    for(uint8_t i = 0; i < n_peripherals; i++){
      if(i2c_peripherals[i]->addr == p_event->xfer_desc.address){
        i2c_peripherals[i]->evt_handler(p_event, p_context);
      }
    }
}

bool i2c_init(void){
  ret_code_t err_code;

    const nrf_drv_twi_config_t twi0_config = {
       .scl                = TWI0_I2C_SCL_PIN,
       .sda                = TWI0_I2C_SDA_PIN,
       .frequency          = I2C_FREQ,
       .interrupt_priority = I2C_IRQ_PRI,
       .clear_bus_init     = false
    };

     NRF_GPIO->PIN_CNF[TWI0_I2C_SCL_PIN] =     \
    (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
  | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos) \
  | (NRF_GPIO_PIN_NOPULL    	 << GPIO_PIN_CNF_PULL_Pos)  \
  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos) \
  | (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos);

    err_code = nrf_drv_twi_init(&m_twi[0], &twi0_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi[0]);

    const nrf_drv_twi_config_t twi1_config = {
       .scl                = TWI1_I2C_SCL_PIN,
       .sda                = TWI1_I2C_SDA_PIN,
       .frequency          = I2C_FREQ,
       .interrupt_priority = I2C_IRQ_PRI,
       .clear_bus_init     = false
    };

    NRF_GPIO->PIN_CNF[TWI1_I2C_SCL_PIN] =     \
    (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) \
  | (GPIO_PIN_CNF_DRIVE_S0S1     << GPIO_PIN_CNF_DRIVE_Pos) \
  | (NRF_GPIO_PIN_NOPULL    	 << GPIO_PIN_CNF_PULL_Pos)  \
  | (GPIO_PIN_CNF_INPUT_Disconnect  << GPIO_PIN_CNF_INPUT_Pos) \
  | (GPIO_PIN_CNF_DIR_Output      << GPIO_PIN_CNF_DIR_Pos);

    err_code = nrf_drv_twi_init(&m_twi[1], &twi1_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi[1]);
  
    return err_code == NRF_SUCCESS;
}

bool i2c_deinit(void){
  nrf_drv_twi_uninit(&m_twi[0]);
  nrf_drv_twi_uninit(&m_twi[1]);
  return true;
}

bool i2c_register_peripheral(i2c_peripheral_t *p){
  if(n_peripherals >= MAX_I2C_PERIPHERALS) return false;
  i2c_peripherals[n_peripherals++] = p;
  return true;
}

bool i2c_write(i2c_peripheral_t *p, uint8_t reg_addr, const uint8_t * const data, uint8_t length){
  ret_code_t ret = NRF_SUCCESS;
  *m_buffer = reg_addr;
  memcpy(m_buffer + sizeof(reg_addr), data, length);
  m_xfer_done = false;
  ret = nrf_drv_twi_tx(&m_twi[p->twi_instance_id], p->addr, m_buffer, 1 + length, 0);
  
  while(!m_xfer_done){
    __WFE();
  }
  
  APP_ERROR_CHECK(ret); 
  
  return ret == NRF_SUCCESS;
}

bool i2c_read(i2c_peripheral_t *p, uint8_t reg_addr, uint8_t *data, uint8_t length){
  ret_code_t ret = NRF_SUCCESS;
  
  m_xfer_done = false;
  ret = nrf_drv_twi_tx(&m_twi[p->twi_instance_id], p->addr, &reg_addr, sizeof(reg_addr), true);
  if(ret != NRF_SUCCESS){
    NRF_LOG_RAW_INFO("[i2c] Read error: 0x%x\n", ret);
  }
  APP_ERROR_CHECK(ret);
  
  while (!m_xfer_done){
    __WFE();
  }

  m_xfer_done = false;
  ret = nrf_drv_twi_rx(&m_twi[p->twi_instance_id], p->addr, data, length);
  APP_ERROR_CHECK(ret);
  while (!m_xfer_done){
    __WFE();
  }
  
  return ret == NRF_SUCCESS;
}

uint16_t i2c_read16(i2c_peripheral_t *p, uint8_t reg_addr){
  uint8_t data[2];
  i2c_read(p, reg_addr, data, 2);
  return (data[0] << 8) + data[1];
}

bool i2c_write16(i2c_peripheral_t *p, uint8_t reg_addr, uint16_t val){
  uint8_t data[2] = {MSB_16(val), LSB_16(val)};
  return i2c_write(p, reg_addr, data, 2);
}
