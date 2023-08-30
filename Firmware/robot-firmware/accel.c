#include "accel.h"
#include "i2c.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "sensors.h"
#include "nrf_log.h"
#include "event.h"
#include <math.h>

// This shouldn't be here?
#include "ble_sss.h"

#define ACCEL_DEBUG 0
#define ACCEL_BIAS_SAMPLES 40
#define COLLISION_THRESHOLD_MOVING 3
#define COLLISION_THRESHOLD_STATIONARY 500
#define SAMPLE_RATE_DEFAULT 0x13 // 100 Hz
#define SAMPLE_RATE_COLLISION_DETECT 0x17 // 1000 Hz
#define FS_DEFAULT ACCEL_RANGE_8G
                                    // 2g  ,  4g  ,  8g  ,  12g  ,  16g  
const float accel_conversion_mg[5] = {0.061, 0.122, 0.244, 0.366, 0.488};
const uint8_t accel_i2c_val[5] =     {0b000, 0b001, 0b010, 0b100, 0b011};
static uint8_t accel_range;
static uint8_t status;
float accel_x, accel_y, accel_z;
float raw[3], praw[3];
static float accel_xbias, accel_ybias, accel_zbias;
static float alpha = 0.5;
static volatile bool bias_set;
static accel_collision_detection_t m_detect_collisions;
static uint64_t sample_count;

static i2c_peripheral_t m_i2c;
static sensor_t m_accel;

#define ACCEL_STREAM_DATA_LENGTH 64
static uint8_t m_data_buffer[ACCEL_STREAM_DATA_LENGTH];

static void evt_handler(nrf_drv_twi_evt_t const * p_event, void * p_context){
    switch (p_event->type){
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX){
                // Do something
            }
            break;
        default:
            break;
    }
}

accel_collision_detection_t accel_get_collision_detection_state(void){
  return m_detect_collisions;
}

sensor_t* accel_get_sensor_class(void){
  return &m_accel;
}

void accel_update_current_measurements(void){
  uint8_t acc[6];
  i2c_read(&m_i2c, MC34X9_REG_XOUT_LSB, acc, 6);
  
  raw[0] = ((int16_t) ((acc[1] << 8) + acc[0])) * accel_conversion_mg[accel_range];
  if(bias_set) {
    raw[0] -= accel_xbias;
  }

  raw[1] = ((int16_t) ((acc[3] << 8) + acc[2])) * accel_conversion_mg[accel_range];
  if(bias_set) {
    raw[1] -= accel_ybias;
  }

  raw[2] = ((int16_t) ((acc[5] << 8) + acc[4])) * accel_conversion_mg[accel_range];
  if(bias_set) {
    raw[2] -= accel_zbias;
  }

  // Post-processing
  // Roll should be between -2.5 & 2.5 degrees (Maybe upper bound can be tighter)
  //float roll = atan2(yval, zval);
  //if(fabs(roll) >= 0.05){
  //  if(roll < -0.05){
  //    yval = - 0.05 * zval - yval;
  //  }else{
  //    yval = + 0.05 * zval - yval;
  //  }
  //}
  //// If linear acceleration is too large (> 10cm/s^2), discard
  //if(yval > 100){
  //  xval = accel_x;
  //  yval = accel_y;
  //  zval = accel_z;
  //}
  
  if(sample_count > 0){
    accel_x = (1 - alpha) * accel_x + alpha * raw[0];
    accel_y = (1 - alpha) * accel_y + alpha * raw[1];
    accel_z = (1 - alpha) * accel_z + alpha * raw[2];
  }else{
    accel_x = raw[0];
    accel_y = raw[1];
    accel_z = raw[2];
  }

  float rawdiff[3];
  for(int i =0; i < 3; i++) rawdiff[i] = raw[i] - praw[i];
  //float mag = sqrtf(rawdiff[0] * rawdiff[0] + rawdiff[1] * rawdiff[1]);

  if(m_detect_collisions != COLLISIONS_DETECT_OFF){
    //float pmag = (praw[0] * praw[0] + praw[1] * praw[1])/1e6;
    float mag = (accel_x * accel_x + accel_y * accel_y)/1e6;

    float diffmag = sqrtf(rawdiff[0] * rawdiff[0] + rawdiff[1] * rawdiff[1]);
    
    //float magdiff = (mag/1e6 - pmag/1e6) * (mag/1e6 - pmag/1e6);
    
    if(m_detect_collisions == COLLISIONS_DETECT_MOVING && 
       mag > COLLISION_THRESHOLD_MOVING){
        eventQueuePush(EVENT_COLLISION);
    }else if(m_detect_collisions == COLLISIONS_DETECT_STATIONARY && 
       diffmag > COLLISION_THRESHOLD_STATIONARY){
        eventQueuePush(EVENT_COLLISION); 
    }
  }

  memcpy(praw, raw, sizeof(float) * 3);

  ((float *) m_data_buffer)[0] = accel_x;
  ((float *) m_data_buffer)[1] = accel_y;
  ((float *) m_data_buffer)[2] = accel_z;

  //((float *)m_data_buffer)[0] = rawdiff[0];
  //((float *)m_data_buffer)[1] = rawdiff[1];
  //((float *)m_data_buffer)[2] = rawdiff[2];

#if ACCEL_DEBUG
  //NRF_LOG_RAW_INFO("%08d \n", systemTimeGetUs());
  NRF_LOG_RAW_INFO("acc_x: " NRF_LOG_FLOAT_MARKER,
                    NRF_LOG_FLOAT(accel_x));
  NRF_LOG_RAW_INFO(" acc_y: " NRF_LOG_FLOAT_MARKER,
                    NRF_LOG_FLOAT(accel_y));
  NRF_LOG_RAW_INFO(" acc_z: " NRF_LOG_FLOAT_MARKER "\n",
                    NRF_LOG_FLOAT(accel_z));
#endif  

  // FIXME: Might cause stalling if ble calibrate triggers while incrementing
  sample_count++;

  if(!bias_set){
    accel_xbias = accel_x;
    accel_ybias = accel_y;
    accel_zbias = accel_z;
    if(sample_count >= ACCEL_BIAS_SAMPLES){
      NRF_LOG_INFO("ACCEL DATA CALIBRATED");
      accel_update_biases();
      eventQueuePush(EVENT_ACCEL_CALIBRATION_DONE);
    }
  }

  m_accel.data_length = 3 * sizeof(float);
}

void accel_cfg_handler(const uint8_t * const cfg_data, uint16_t len){
  ASSERT((cfg_data[1] & (1 << SENSOR_ACCEL_ID)) > 0 );
  
  switch(cfg_data[0]){
    case SSS_STREAM_ENABLE:
      //accel_update_current_measurements();
      eventQueuePush(EVENT_ACCEL_INTERRUPT);
    break;
    default:
    break;
  }
}

void accel_clear_interrupts(){
  // Clear all interrupts
  status = 0;
  i2c_write(&m_i2c, MC34X9_REG_INTR_STAT, &status, 1); 
}

void accel_handle_interrupt(void){
  i2c_read(&m_i2c, MC34X9_REG_INTR_STAT, &status, 1);
  
  //NRF_LOG_RAW_INFO("Accel Interrupt Handled!\n")
  accel_clear_interrupts();
  
  if(status & (1 << ACCEL_AM_INT_BIT)){
    // Anymotion event
    NRF_LOG_RAW_INFO("[accel] AM EVENT!\n");
   eventQueuePush(EVENT_ACCEL_ANYMOTION);
  }else{
    accel_update_current_measurements();
  }
}

void accel_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
  //NRF_LOG_RAW_INFO("Accel Interrupt!\n");
  eventQueuePush(EVENT_ACCEL_INTERRUPT);
}

void accel_set_state(accel_state_t state){
  uint8_t d;
  i2c_read(&m_i2c, MC34X9_REG_MODE, &d, 1);
  d &= 0b11001100;
  d |= (state & 0xFF);
  i2c_write(&m_i2c, MC34X9_REG_MODE, &d, 1);

  //uint8_t t;
  //i2c_read(&m_i2c, MC34X9_REG_MODE, &t, 1);
  //APP_ERROR_CHECK_BOOL(d == t);
}


void accel_disable_collision_detection(void){
  m_detect_collisions = COLLISIONS_DETECT_OFF;
  
  uint8_t regval;
  regval = SAMPLE_RATE_DEFAULT; // Set sample rate to default
  i2c_write(&m_i2c, MC34X9_REG_SR, &regval, 1);
}

void accel_enable_collision_detection(bool moving){
  if(moving){
    m_detect_collisions = COLLISIONS_DETECT_MOVING;
  }else{
    m_detect_collisions = COLLISIONS_DETECT_STATIONARY;
  }
  accel_set_state(ACCEL_STATE_STANDBY);
  accel_clear_interrupts();
  uint8_t regval;
  regval = SAMPLE_RATE_COLLISION_DETECT; // Set sample rate to 1000Hz
  i2c_write(&m_i2c, MC34X9_REG_SR, &regval, 1);
  
  accel_set_state(ACCEL_STATE_WAKE);
}


bool accel_init(void){
  sample_count = 0;
  
  m_i2c.addr = MC34X9_SLAVE_ADDR;
  m_i2c.evt_handler = evt_handler;
  m_i2c.twi_instance_id = 0;

  if (i2c_register_peripheral(&m_i2c)){
    m_accel.on_state = true;
    m_accel.cfg_handler = accel_cfg_handler;
    m_accel.calibrate = accel_reset_biases;
    m_accel.sensor_data = m_data_buffer;
    m_accel.data_length = ACCEL_STREAM_DATA_LENGTH;
    m_accel.stream_enabled = false;
    m_accel.id = SENSOR_ACCEL_ID;

    ret_code_t err_code;
    
    // Configure accelerometer
    accel_set_state(ACCEL_STATE_STANDBY);
    uint8_t regval;
    
    // Enable push-pull mode on interrupt pin INT1
    i2c_read(&m_i2c, MC34X9_REG_GPIO_CTRL, &regval, 1);
    regval &= 0b00110011;
    regval |= 0b00001000;
    i2c_write(&m_i2c, MC34X9_REG_GPIO_CTRL, &regval, 1);

    uint8_t gpio_ctrl_test;
    i2c_read(&m_i2c, MC34X9_REG_GPIO_CTRL, &gpio_ctrl_test, 1);
    //APP_ERROR_CHECK_BOOL(gpio_ctrl_test == regval);

    // Enable acquisition interrupts only
    i2c_read(&m_i2c, MC34X9_REG_INTR_CTRL, &regval, 1);
    regval &= 0b00100000;
    regval |= 0b10000000;
    i2c_write(&m_i2c, MC34X9_REG_INTR_CTRL, &regval, 1);
    
    uint8_t intr;
    i2c_read(&m_i2c, MC34X9_REG_INTR_CTRL, &intr, 1);
    //APP_ERROR_CHECK_BOOL(intr == regval);

    i2c_read(&m_i2c, MC34X9_REG_SR, &regval, 1);
    regval = SAMPLE_RATE_DEFAULT; // Set sample rate to Default
    //regval = SAMPLE_RATE_COLLISION_DETECT; // Set sample rate to Default
    i2c_write(&m_i2c, MC34X9_REG_SR, &regval, 1);
    
    
    i2c_read(&m_i2c, MC34X9_REG_FIFO_CTRL_SR2, &regval, 1);
    //regval = 0b00000100; // Decimate by 8
    regval = 0b00000001; // Decimate by 2
    i2c_write(&m_i2c, MC34X9_REG_FIFO_CTRL_SR2, &regval, 1);

    // Set accel range to be default
    accel_range = FS_DEFAULT;
    i2c_read(&m_i2c, MC34X9_REG_RANGE_C, &regval, 1);
    regval = 0;
    regval |= accel_i2c_val[accel_range] << (4);
    regval |= 0b00001001; // Enable LPF @ IDR/4.255;
    i2c_write(&m_i2c, MC34X9_REG_RANGE_C, &regval, 1);

    // Clear interrupts
    accel_clear_interrupts();

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(ACCEL_INT_PIN, &in_config, accel_int_handler);
    APP_ERROR_CHECK(err_code);

    // Enable interrupts
    nrf_drv_gpiote_in_event_enable(ACCEL_INT_PIN, true);
    
    // Start sampling
    accel_set_state(ACCEL_STATE_WAKE);

    NRF_LOG_RAW_INFO("Accel interrupt state %d\n", nrf_gpio_pin_read(ACCEL_INT_PIN));

    accel_reset_biases();

    return true;
  }

  return false;
}

bool accel_deinit(void){
  accel_set_state(ACCEL_STATE_STANDBY);
  nrf_drv_gpiote_in_event_disable(ACCEL_INT_PIN);
  nrf_drv_gpiote_in_uninit(ACCEL_INT_PIN);
}

bool accel_test(void){
  uint8_t byte;
  if(i2c_read(&m_i2c, MC34X9_REG_RD_CNT, &byte, 1)){
    return byte == 0x06; // Default value
  }
  return false;
}

bool accel_stream_enabled(void){
  return m_accel.stream_enabled;
}

// Careful, different sources define pitch & roll differently.
// Roll is around x-axis
float accel_get_roll(void){
  return atan2(accel_y, accel_z);
}

// Pitch is around y-axis
float accel_get_pitch(void){
  return atan2((- accel_x) , sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / M_PI;
}

void accel_get_xyz(float* arr){
  arr[0] = accel_x;
  arr[1] = accel_y;
  arr[2] = accel_z;
}

void accel_reset_biases(void){
  bias_set = false;
  sample_count = 0;
  accel_x = 0;
  accel_y = 0;
  accel_z = 1000;
}

void accel_update_biases(void){
  bias_set = true;
  accel_xbias = accel_x;
  accel_ybias = accel_y;
  accel_zbias = accel_z;
  accel_x = 0;
  accel_y = 0;
  accel_z = 1000;
  accel_zbias -= 1000;
}

volatile bool accel_ready(void){
  return bias_set;
}