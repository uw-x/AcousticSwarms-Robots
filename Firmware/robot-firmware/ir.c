#include <stdbool.h>
#include "ir.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "event.h"
#include "nrf_log.h"
#include "timers.h"
#include "led.h"


static volatile ir_sensors_status_t previous_status;
static volatile ir_sensors_status_t status;
static volatile bool m_debouncing;
static int8_t marker_count;
static ir_mode mode;
static bool first_sample;
static bool enabled;
static bool streaming;

void ir_timer_init(void){
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos; // Ensure the timer uses 24-bit bitmode or higher
  NRF_TIMER4->PRESCALER = 0;                                                      // Set the prescaler to 0, for a timer interval of 1/16 us (16M / 2^4)
  NRF_TIMER4->CC[0] = 16 * 1000 * 10;                                                   // Set the CC[0] register to hit after 0.01 seconds
  NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Msk;                             // Trigger the interrupt when reaching CC[0]
  NVIC_SetPriority(TIMER4_IRQn, 5);                                         // Set a medium IRQ priority and enable interrupts for the timer module
  NVIC_EnableIRQ(TIMER4_IRQn);
}

void ir_timer_deinit(void){
  NVIC_DisableIRQ(TIMER4_IRQn);
}

void ir_debounce(void){
  NRF_TIMER4->TASKS_CLEAR = 1;                                                  // Clear and start the timer
  NRF_TIMER4->TASKS_START = 1;
}

void ir_update_status(void){
  status = (nrf_drv_gpiote_in_is_set(SENSOR_LEFT) << 1) | nrf_drv_gpiote_in_is_set(SENSOR_RIGHT);
}

void TIMER4_IRQHandler(void)
{
  if(NRF_TIMER4->EVENTS_COMPARE[0]) {
    NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    NRF_TIMER4->TASKS_STOP = 1;
    NRF_LOG_RAW_INFO("Timer 4 interrupted!\n");
    previous_status = status;
    ir_update_status();
    if(previous_status != status){
      eventQueuePush(EVENT_IR_STATUS_UPDATED);
    }
    m_debouncing = false;
  }
}


/**
 * @brief Function for changing the status variable in response to the sensor interrupts.
 *
 */
void sensor_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  if(!m_debouncing){
    //NRF_LOG_RAW_INFO("%08d [ir] IR INTERRUPT!\n", systemTimeGetUs());
    m_debouncing = true;
    ir_debounce();
  }
}

/**
 * @brief Function for initializing leds for debugging.
 *
// */
//void led_init(void) {
//  ret_code_t err_code;
//  nrf_drv_gpiote_out_config_t out_config1 = GPIOTE_CONFIG_OUT_SIMPLE(false);
//  err_code = nrf_drv_gpiote_out_init(PIN_OUT_LEFT, &out_config1);
//  APP_ERROR_CHECK(err_code);
//  nrf_drv_gpiote_out_config_t out_config2 = GPIOTE_CONFIG_OUT_SIMPLE(false);
//  err_code = nrf_drv_gpiote_out_init(PIN_OUT_RIGHT, &out_config2);
//  APP_ERROR_CHECK(err_code);
//}

bool ir_init(void) {
  
  ret_code_t err_code, err_code_left, err_code_right;
  
  if (!nrfx_gpiote_is_init()){
    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);
  }
  // Used for debugging.
 // led_init();

  nrf_drv_gpiote_in_config_t in_config_sensor_left = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config_sensor_left.pull = NRF_GPIO_PIN_PULLUP;

  err_code_left = nrf_drv_gpiote_in_init(SENSOR_LEFT, &in_config_sensor_left, sensor_handler);

  nrf_drv_gpiote_in_config_t in_config_sensor_right = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
  in_config_sensor_right.pull = NRF_GPIO_PIN_PULLUP;

  err_code_right = nrf_drv_gpiote_in_init(SENSOR_RIGHT, &in_config_sensor_right, sensor_handler);

  nrf_gpio_cfg_output(IR_ENABLE_PIN);

  ir_timer_init();
  m_debouncing = false;
  first_sample = true;
  enabled = false;
  ir_disable();

  return (err_code_left == NRFX_SUCCESS && err_code_right == NRFX_SUCCESS);
}

void ir_enable(bool ignore_first) {
  nrf_gpio_pin_set(IR_ENABLE_PIN);
  delayUs(200);
  ir_update_status();
  NRF_LOG_RAW_INFO("[ir] ENABLED! CURRENT STATUS: %d\n", status);
  nrf_drv_gpiote_in_event_enable(SENSOR_LEFT, true);
  nrf_drv_gpiote_in_event_enable(SENSOR_RIGHT, true);
  enabled = true;
  first_sample = ignore_first;
}

void ir_set_mode(ir_mode _mode){
  mode = _mode;
  marker_count = 0;
}

bool ir_enabled(){
  return enabled;
}

ir_mode ir_get_mode(void){
  return mode;
}

bool ir_target_depth_reached(void){
  return marker_count <= 0;
}

void ir_set_target_depth(uint8_t depth){
  marker_count = depth;
}

void ir_increment_marker_count(void){
  marker_count--;
  led_rgb(marker_count & 4, marker_count & 2, marker_count & 1);
  NRF_LOG_RAW_INFO("MARKERS LEFT: %d\n", marker_count);
}

void ir_disable(void) {
  nrf_gpio_pin_clear(IR_ENABLE_PIN);
  nrfx_gpiote_in_event_disable(SENSOR_LEFT);
  nrfx_gpiote_in_event_disable(SENSOR_RIGHT);
  mode = IR_NONE;
  first_sample = true;
  enabled = false;
  NRF_LOG_RAW_INFO("[ir]DISABLED\n");
}

volatile ir_sensors_status_t ir_get_current_status(void) {
  return status;
}

volatile ir_sensors_status_t ir_get_previous_status(void){
  return previous_status;
}

void ir_enable_stream(bool status){
  streaming = status;
}

bool ir_streaming_enabled(void){
  return streaming;
}