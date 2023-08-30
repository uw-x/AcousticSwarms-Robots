#include "fuel_gauge.h"
#include "i2c.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "sensors.h"
#include "nrf_log.h"
#include "event.h"

// This shouldn't be here?
#include "ble_sss.h"


static i2c_peripheral_t m_i2c;
static sensor_t m_fg;
static uint16_t status, config;
static float vcell, soc, crate;
static float motor_compensation;

#define FG_STREAM_DATA_LENGTH 32
static uint8_t m_data_buffer[FG_STREAM_DATA_LENGTH];

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

sensor_t *fg_get_sensor_class(void) {
  return &m_fg;
}


float _fg_get_current_battery_level(void) {
  vcell = i2c_read16(&m_i2c, MAX17048_VCELL) * 78.125e-6;
  motor_compensation = 1 + (4.2 - vcell) * 0.15;
  return vcell;
}

float fg_get_current_battery_level(){
  return vcell;
}

float fg_get_motor_compensation(void){
  return motor_compensation;
}

float _fg_get_current_soc(void) {
  soc = i2c_read16(&m_i2c, MAX17048_SOC) * 0.00390625;
  return soc;
}

float fg_get_current_soc(){
  return soc;
}

float _fg_get_current_crate(void) {
  crate = ((int16_t) i2c_read16(&m_i2c, MAX17048_CRATE)) * 0.208;
  return crate;
}

float fg_get_current_crate(){
  return crate;
}

void fg_update_current_measurements(void){
  soc = _fg_get_current_soc();
  vcell = _fg_get_current_battery_level();
  crate = _fg_get_current_crate();

  ((float *)m_data_buffer)[0] = vcell;
  ((float *)m_data_buffer)[1] = soc;
  ((float *)m_data_buffer)[2] = crate;
  m_fg.data_length = 3 * sizeof(float);
}

void fg_cfg_handler(const uint8_t *const cfg_data, uint16_t len) {
  ASSERT((cfg_data[1] & (1 << SENSOR_FG_ID)) > 0 );

  switch(cfg_data[0]){
    case SSS_STREAM_ENABLE:
      //fg_update_current_measurements();
      eventQueuePush(EVENT_FUEL_GAUGE_INTERRUPT);
    break;
    default:
    break;
  }
}

void fg_clear_interrupts(bool read_regs){
  if(read_regs){
    config = i2c_read16(&m_i2c, MAX17048_CONFIG);
    status = i2c_read16(&m_i2c, MAX17048_STATUS);
  }

  config &= (~(1 << MAX17048_CONFIG_ALRT_BIT));
  
  // Clear alert flag
  i2c_write16(&m_i2c, MAX17048_CONFIG, config);

  // Clear all interrupts
  status &= (~(0b00111110 << MAX17048_SC_INT_BIT));
  i2c_write16(&m_i2c, MAX17048_STATUS, status); 
}

void fg_handle_interrupt(void){
  config = i2c_read16(&m_i2c, MAX17048_CONFIG);
  
  if(config & (1 << MAX17048_CONFIG_ALRT_BIT)){
    status = i2c_read16(&m_i2c, MAX17048_STATUS);
    if(status & (1 << MAX17048_SC_INT_BIT)){
      // Handle changed by 1 %
      NRF_LOG_INFO("Battery SOC changed by 1%!");
      //led_toggle(0,0,1);
      
    }
    fg_clear_interrupts(false);
  }
  
  fg_update_current_measurements();
}

void fg_int_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action){
  eventQueuePush(EVENT_FUEL_GAUGE_INTERRUPT);
}

bool fg_init() {
  m_i2c.addr = MAX17048_ADDR_SLAVE;
  m_i2c.evt_handler = evt_handler;
  m_i2c.twi_instance_id = 0;

  if (i2c_register_peripheral(&m_i2c)) {
    ret_code_t err_code;
    m_fg.on_state = true;

    // Initialize sensor struct
    m_fg.cfg_handler = fg_cfg_handler;
    m_fg.sensor_data = m_data_buffer;
    m_fg.data_length = FG_STREAM_DATA_LENGTH;
    m_fg.stream_enabled = false;
    m_fg.id = SENSOR_FG_ID;

    // Initialize interrupt
    // Interrupt pin is active low, so check for High to Low transitions

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

    // Enable interrupts on 1% battery change
    uint16_t cfg = i2c_read16(&m_i2c, MAX17048_CONFIG);
    
    cfg |= (1 << MAX17048_CONFIG_ALSC_BIT);
    i2c_write16(&m_i2c, MAX17048_CONFIG, cfg);

    fg_clear_interrupts(true);

    cfg = i2c_read16(&m_i2c, MAX17048_CONFIG);
    err_code = nrf_drv_gpiote_in_init(FG_INT_PIN, &in_config, fg_int_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(FG_INT_PIN, true);

    return err_code == NRF_SUCCESS;
  }
  return false;
}

uint16_t fg_get_version() {
  uint16_t data = i2c_read16(&m_i2c, MAX17048_VERSION);
  return data;
}

bool fg_test() {
  return fg_get_version() == 0x0012;
}

/*============================================================================*/
void fg_Compensation(uint8_t RComp) {
  uint16_t data = i2c_read16(&m_i2c, MAX17048_CONFIG);
  data &= 0x00FF;
  data |= RComp << 8;
  i2c_write16(&m_i2c, MAX17048_CONFIG, data);
}

/*POR Power on Reset*/
/*============================================================================*/
void fg_Reset() {
  uint16_t value = MAX17048_RESET;
  i2c_write16(&m_i2c, MAX17048_CMD, value);
}

/*============================================================================*/
void fg_SleepEnable() {
  uint16_t value = i2c_read16(&m_i2c, MAX17048_MODE);
  value = (value | (0x0001 << MAX17048_MODE_EN_SLEEP_BIT));
  i2c_write16(&m_i2c, MAX17048_MODE, value);
}

/*============================================================================*/
void fg_QStart() {
  uint16_t value = i2c_read16(&m_i2c, MAX17048_MODE);
  value = (value | (0x0001 << MAX17048_MODE_QUICK_START_BIT));
  i2c_write16(&m_i2c, MAX17048_MODE, value);
}

/*============================================================================*/
void fg_Sleep(uint8_t On_Off) {
  uint16_t value = i2c_read16(&m_i2c, MAX17048_CONFIG);

  if (On_Off) {
    value = value | (0x0001 << MAX17048_CONFIG_SLEEP_BIT);
  } else {
    value = value & (~(0x0001 << MAX17048_CONFIG_SLEEP_BIT));
  }
  i2c_write16(&m_i2c, MAX17048_CONFIG, value);
}

bool fg_stream_enabled(void){
  return m_fg.stream_enabled; 
}