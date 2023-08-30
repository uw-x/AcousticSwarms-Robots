#include "nrf_log.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"

#include "lsm6dso_reg.h"
#include "gyro.h"
#include "i2c.h"
#include "ble_sss.h"
#include "event.h"


#define GYRO_DEBUG 0
#define GYRO_BIAS_SAMPLES 40

static i2c_peripheral_t m_i2c;
static float gyro_x, gyro_y, gyro_z;
static float gyro_xbias, gyro_ybias, gyro_zbias;
static volatile bool bias_set = false;
static volatile uint64_t sample_count;

static sensor_t m_gyro;
const float gyro_conversion[3] = {1/1024.0, 1/4096.0, 1/16384.0};
static float alpha = 0.9;
static stmdev_ctx_t m_dev;

#define GYRO_STREAM_DATA_LENGTH 64

static uint8_t m_data_buffer[GYRO_STREAM_DATA_LENGTH];

static void evt_handler(nrf_drv_twi_evt_t const *p_event, void *p_context) {
  switch (p_event->type) {
  case NRF_DRV_TWI_EVT_DONE:
    if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX) {
      // Do something
    }
    break;
  default:
    break;
  }
}

void gyro_cfg_handler(const uint8_t * const cfg_data, uint16_t len){
  ASSERT((cfg_data[1] & (1 << SENSOR_GYRO_ID)) > 0 );
  
  switch(cfg_data[0]){
    case SSS_STREAM_ENABLE:
      eventQueuePush(EVENT_GYRO_INTERRUPT);
    break;
    default:
    break;
  }
}

void gyro_update_current_measurements(void){
  int16_t gyro[3];
  //ism330dlc_angular_rate_raw_get(&m_dev, gyro);
  lsm6dso_angular_rate_raw_get(&m_dev, gyro);  
  float xval, yval, zval;
  xval = lsm6dso_from_fs1000_to_mdps(gyro[0]);
  if(bias_set) xval -= gyro_xbias;
  
  
  yval = lsm6dso_from_fs1000_to_mdps(gyro[1]);
  if(bias_set) yval -= gyro_ybias;
  
  
  zval = lsm6dso_from_fs1000_to_mdps(gyro[2]);
  if(bias_set) zval -= gyro_zbias;

  if(sample_count > 0){
    gyro_x = (1 - alpha) * gyro_x + alpha * xval;
    gyro_y = (1 - alpha) * gyro_y + alpha * yval;
    gyro_z = (1 - alpha) * gyro_z + alpha * zval;
  }else{
    gyro_x = xval;
    gyro_y = yval;
    gyro_z = zval;
  }

#if GYRO_DEBUG
  NRF_LOG_RAW_INFO("gx: " NRF_LOG_FLOAT_MARKER,
                    NRF_LOG_FLOAT(gyro_x));
  NRF_LOG_RAW_INFO(" gy: " NRF_LOG_FLOAT_MARKER,
                    NRF_LOG_FLOAT(gyro_y));
  NRF_LOG_RAW_INFO(" gz: " NRF_LOG_FLOAT_MARKER "\n",
                    NRF_LOG_FLOAT(gyro_z));
#endif 
  
  // FIXME: Might cause stalling if ble calibrate triggers while incrementing
  sample_count++;

  if(!bias_set){
    gyro_xbias = (gyro_xbias + gyro_x) / 2;
    gyro_ybias = (gyro_ybias + gyro_y) / 2;
    gyro_zbias = (gyro_zbias + gyro_z) / 2;
    if(sample_count >= GYRO_BIAS_SAMPLES){
      NRF_LOG_INFO("GYRO DATA CALIBRATED");
      bias_set = true;
      gyro_x = 0;
      gyro_y = 0;
      gyro_z = 0;
      eventQueuePush(EVENT_GYRO_CALIBRATION_DONE);
    }
  }

  ((float *) m_data_buffer)[0] = gyro_x;
  ((float *) m_data_buffer)[1] = gyro_y;
  ((float *) m_data_buffer)[2] = gyro_z;
  m_gyro.data_length = 3 * sizeof(float);
}

float gyro_get_yaw_rate(void){
  return gyro_z;
}

float gyro_get_pitch_rate(void){
  return gyro_x;
}

void gyro_clear_interrupts(){
  // Clear all interrupts
  gyro_update_current_measurements();
  //i2c_write(&m_i2c, ISM330DLC_STATUS_REG, &status, 1); 
}

void gyro_handle_interrupt(void){
  lsm6dso_all_sources_t all_source;
  lsm6dso_all_sources_get(&m_dev, &all_source);
  if(all_source.drdy_g){
    gyro_update_current_measurements();
  }
  //gyro_clear_interrupts();
}

void gyro_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
  //NRF_LOG_RAW_INFO("Gyro Interrupt!\n");
  eventQueuePush(EVENT_GYRO_INTERRUPT);
}

int32_t platform_write(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len){
  return i2c_write((i2c_peripheral_t*) handle, Reg, Bufp, len) == 0;
}

int32_t platform_read(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len){
  return i2c_read((i2c_peripheral_t*) handle, Reg, Bufp, len) == 0;
}

bool gyro_init(void){
  gyro_xbias = 0;
  gyro_ybias = 0;
  gyro_zbias = 0;
  sample_count = 0;

  m_i2c.addr = GYRO_ADDR;
   m_i2c.evt_handler = evt_handler;
   m_i2c.twi_instance_id = 1;

   if(i2c_register_peripheral(&m_i2c)){
      m_gyro.on_state = true;
      m_gyro.cfg_handler = gyro_cfg_handler;
      m_gyro.calibrate = gyro_reset_biases;
      m_gyro.sensor_data = m_data_buffer;
      m_gyro.data_length = GYRO_STREAM_DATA_LENGTH;
      m_gyro.stream_enabled = false;
      m_gyro.id = SENSOR_GYRO_ID;

      m_dev.write_reg = platform_write;
      m_dev.read_reg = platform_read;
      m_dev.handle = (void*) &m_i2c;

      /* Restore default configuration */
      lsm6dso_reset_set(&m_dev, PROPERTY_ENABLE);
      uint8_t rst;
      do {
        lsm6dso_reset_get(&m_dev, &rst);
      } while (rst);

      /* Disable I3C interface */
      lsm6dso_i3c_disable_set(&m_dev, LSM6DSO_I3C_DISABLE);

      //// Set gyro full scale to 250 dps
      //ism330dlc_gy_full_scale_set(&m_dev, ISM330DLC_250dps);
      // Set gyro full scale to 500 dps
      
      //ism330dlc_gy_full_scale_set(&m_dev, ISM330DLC_500dps);
      //lsm6dso_gy_full_scale_set(&m_dev, LSM6DSO_500dps);
      lsm6dso_gy_full_scale_set(&m_dev, LSM6DSO_1000dps);

      // Set gyro ODR to 104Hz
      //ism330dlc_gy_data_rate_set(&m_dev, ISM330DLC_GY_ODR_104Hz);
      lsm6dso_gy_data_rate_set(&m_dev, LSM6DSO_GY_ODR_104Hz);

      uint8_t dev_id;
      lsm6dso_device_id_get(&m_dev, &dev_id);
      //NRF_LOG_RAW_INFO("Device id: %d\n", dev_id);

      // Enable Gyro 
      nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
      in_config.pull = NRF_GPIO_PIN_NOPULL;
      
      ret_code_t err_code = nrf_drv_gpiote_in_init(GYRO_INT_PIN, &in_config, gyro_int_handler);
      APP_ERROR_CHECK(err_code);
    
      // Enable interrupts
      nrf_drv_gpiote_in_event_enable(GYRO_INT_PIN, true);

      lsm6dso_int_mode_t val;
      lsm6dso_interrupt_mode_get(&m_dev, &val);
      val.active_low = 1;
      lsm6dso_interrupt_mode_set(&m_dev, val);
      
      // Interrupts are push-pull  
      //lsm6dso_pin_mode_set(&m_dev, LSM6DSO_PUSH_PULL);

      lsm6dso_pin_int1_route_t int1_route;
      lsm6dso_pin_int1_route_get(&m_dev, &int1_route);
      int1_route.drdy_g = PROPERTY_ENABLE;
      //int1_route.drdy_g = PROPERTY_DISABLE;
      //int1_route.drdy_xl = PROPERTY_ENABLE;
      lsm6dso_pin_int1_route_set(&m_dev, int1_route);
    
      //gyro_timer_init();

      // Clear interrupts
      gyro_clear_interrupts();
      gyro_reset_biases();

     return true;
   }

  return false;
}

bool gyro_deinit(void){
  // TODO
  return true;
}

bool gyro_stream_enabled(void){
  return m_gyro.stream_enabled;
}

sensor_t* gyro_get_sensor_class(void){
  return &m_gyro;
}

uint64_t gyro_get_sample_count(void){
  return sample_count;
}

void gyro_reset_biases(void){
  bias_set = false;
  sample_count = 0;
  gyro_x = 0;
  gyro_y = 0;
  gyro_z = 0;
}

void gyro_update_biases(void){
  gyro_xbias = gyro_x;
  gyro_ybias = gyro_y;
  gyro_zbias = gyro_z;
  gyro_x = 0;
  gyro_y = 0;
  gyro_z = 0;
  bias_set = true;
}

volatile bool gyro_ready(void){
  return bias_set;
}