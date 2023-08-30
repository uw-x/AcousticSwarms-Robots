#include "gpio.h"
#include "led.h"
#include "pin_config.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "app_timer.h"

//APP_TIMER_DEF(m_bsp_leds_tmr);
static led_state_t current_state = LED_OFF;

#if LED_ENABLED
void led_rgb(bool r, bool g, bool b){
  nrf_gpio_pin_write(LED_RED, !r);
  nrf_gpio_pin_write(LED_GREEN, !g);
  nrf_gpio_pin_write(LED_BLUE, !b);
}

void led_toggle(bool r, bool g, bool b){
  if(r) nrf_gpio_pin_toggle(LED_RED);
  if(g) nrf_gpio_pin_toggle(LED_GREEN);
  if(b) nrf_gpio_pin_toggle(LED_BLUE);
}

#else
void led_rgb(bool r, bool g, bool b){
  NRF_LOG_RAW_INFO("[led] LED_ENABLED is false, enable LEDs in led.h if you want LED to work.\n");
}

void led_toggle(bool r, bool g, bool b){
  NRF_LOG_RAW_INFO("[led] LED_ENABLED is false, enable LEDs in led.h if you want LED to work.\n");
}
#endif

void led_init(){
  nrf_gpio_cfg_output(LED_RED);
  nrf_gpio_cfg_output(LED_GREEN);
  nrf_gpio_cfg_output(LED_BLUE);
  
  nrf_gpio_pin_write(LED_RED, 1);
  nrf_gpio_pin_write(LED_GREEN, 1);
  nrf_gpio_pin_write(LED_BLUE, 1);
}

void led_deinit(void){
  led_rgb(0,0,0);
}

ret_code_t led_indicate(led_state_t state){
    uint32_t err_code   = NRF_SUCCESS;
    if(current_state == state) return err_code;
    
    uint32_t next_delay = 0;

    switch(state){
        case LED_OFF:
            led_rgb(0,0,0);
            break;

        case LED_ADVERTISING:
            led_rgb(1, 0, 0);
            break;
        
        case LED_CONNECTED:
          led_rgb(0, 0, 1);
          break;

        case LED_MASTER:
          led_rgb(0,1,0);
          break;    
    }
    current_state = state;
    return err_code;
}