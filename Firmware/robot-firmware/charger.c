#include "charger.h"
#include "nrf_drv_power.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"

#include "event.h"
#include "nrf_log.h"
#include "nav.h"
#include "motors.h"
#include "led.h"
#include "ir.h"
#include "timers.h"
#include "ble_manager.h"
#include "ble_dcs.h"
#include "math.h"


static bool m_is_charging = false;
static station_entry_state_name_t m_entry_state = CHARGER_ENTRY_DONE;
static uint8_t m_second_count_internal = 0;
static bool m_respond_to_interrupts = false;
static bool m_charger_entry_enabled = false;
static int current_intensity, intensity_update, max_val;
static float m_angle1, m_angle2;

void usb_handler(nrf_drv_power_usb_evt_t event){
  switch(event){
    case NRF_DRV_POWER_USB_EVT_DETECTED:
      m_is_charging = true;
      led_rgb(0,1,0);
      eventQueuePush(EVENT_CHARGER_CONNECTED);
      break;
    case NRF_DRV_POWER_USB_EVT_REMOVED:
      m_is_charging = false;
      eventQueuePush(EVENT_CHARGER_DISCONNECTED);
      led_rgb(1,0,1);
      break;
    default:
      //NRF_LOG_RAW_INFO("Unhandled USB event type");
    break;
  }

}

static void init_power_clock(void)
{
    ret_code_t ret;
    /* Initializing power and clock */
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    ret = nrf_drv_power_init(NULL);
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_hfclk_request(NULL);
    nrf_drv_clock_lfclk_request(NULL);
    while (!(nrf_drv_clock_hfclk_is_running() &&
            nrf_drv_clock_lfclk_is_running())){
        /* Just waiting */
    }

    /* Avoid warnings if assertion is disabled */
    UNUSED_VARIABLE(ret);
}

// Need this or else I get runtime errors
void __nrf_usb_handler_do_nothing(nrfx_usbd_evt_t const * p_event){}

void charger_init(void){
    // Enabling this causes compile to work on debug builds but prevents
    // interrupts on XL
    if(!nrf_drv_clock_init_check()){
      init_power_clock();
    }
      
    ret_code_t ret;
    ret = nrf_drv_usbd_init(__nrf_usb_handler_do_nothing);
    APP_ERROR_CHECK(ret);
    static const nrf_drv_power_usbevt_config_t config = {
        .handler = usb_handler
    };
    ret = nrf_drv_power_usbevt_init(&config);
    APP_ERROR_CHECK(ret);

    // TESTING ONLY! REMOVE WHEN RELYING ON CHARGER 
    //int int_pin = NRF_GPIO_PIN_MAP(0, 23);
    //nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    //in_config.pull = NRF_GPIO_PIN_PULLUP;

    //ret_code_t err_code = nrf_drv_gpiote_in_init(int_pin, &in_config, p0_23_interrupt_handler);
    //APP_ERROR_CHECK(err_code);
    
    //nrf_drv_gpiote_in_event_enable(int_pin, true);

}

bool is_charging(void){
  return m_is_charging;
}

void charger_start_station_entry(void){
  m_charger_entry_enabled = 1;

  m_entry_state = CHARGER_ENTRY_BEGIN;
  charger_next_stage();
  
  //m_second_count_internal = 2;
  
  //uint8_t args[sizeof(uint64_t) + sizeof(int16_t)];
  //uint64_t time = 500 * 1000;
  //time = 700 * 1000;
  //((uint64_t *)args)[0] = time;
  //((int16_t *)args)[4] = 200;
  //nav_motion_update(MOTION_BACKWARD, args, sizeof(uint64_t) + sizeof(int16_t));
  //((float *)args)[0] = 270;
  //nav_motion_update(MOTION_ROTATE_ANGLE, args, sizeof(float));

}

void charger_state_update(charger_state_update_t update){
  if(update == CHARGER_UPDATE_STATE_NEXT){
    m_second_count_internal = 0;
    if(m_entry_state == CHARGER_ENTRY_DONE){
      NRF_LOG_RAW_INFO("FINAL STATE REACHED\n");
      m_charger_entry_enabled = false;
      uint8_t byte = 0xFF;
      ble_indicate_status(DCS_STATUS_BASE_ENTERED, &byte, 1); 
      return;
    }
    m_entry_state = m_entry_state + 1;
    NRF_LOG_RAW_INFO("NEXT STATE\n");

    switch(m_entry_state){
      //case CHARGER_ENTRY_OUTSIDE:
      //  NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY OUTSIDE\n");
      //  ir_set_mode(IR_DETECT_NONE);
      //break;
      case CHARGER_ENTRY_ROTATION0:{
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY ROTATION 0\n");
        led_rgb(1,0,1);
        ir_set_mode(IR_MODE_CHARGING_STATION_ENTRY);
        current_intensity = 130;
        intensity_update = 10;
        max_val = 200;
        ir_enable(false);
        eventQueuePush(EVENT_IR_STATUS_UPDATED);
      }
      break;
      
      case CHARGER_ENTRY_ROTATION1:{
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY ROTATION 1\n");

        action_t action = {
          .type = NAV_ACTION_RESPONSIVE
        };
        nav_set_current_action(action);

        charger_set_internal_count(2);

        NRF_LOG_RAW_INFO("[charger] UDAPTED SECOND COUNT %d\n", m_second_count_internal);
 
        led_rgb(1,0,0);
        ir_set_mode(IR_DETECT_RIGHT);
        ir_enable(false);

        // Set update parameters;
        current_intensity = 130;
        intensity_update = 10;
        max_val = 200;
  
        motors_rotate_cw(current_intensity);
        eventQueuePush(EVENT_IR_STATUS_UPDATED);
      break;

      case CHARGER_ENTRY_ROTATION2:
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY ROTATION 2\n");
        motors_hard_brake();

        m_angle1 = nav_get_current_state().rotation;
        NRF_LOG_RAW_INFO("[charger] ANGLE 1 %d\n", (int) m_angle1);

        led_rgb(0,1,0);
        ir_set_mode(IR_DETECT_LEFT);
        ir_enable(false);
    
        // Set update parameters;
        current_intensity = 130;
        intensity_update = 10;
        max_val = 200;
        charger_set_internal_count(2);
        motors_rotate_ccw(current_intensity);
        eventQueuePush(EVENT_IR_STATUS_UPDATED);
      }
      break;

      case CHARGER_ENTRY_ROTATION3:
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY ROTATION 3\n");
        led_rgb(1,1,0);
        motors_hard_brake();
        nav_end();
        m_angle2 = nav_get_current_state().rotation;
        charger_set_internal_count(2);

        NRF_LOG_RAW_INFO("[charger] ANGLE 2 %d\n", (int) m_angle2);
        
        // Compute angular mean
        point a1 = {
          .x = cos(m_angle1 * M_PI / 180),
          .y = sin(m_angle1 * M_PI / 180),
        };

        point a2 = {
          .x = cos(m_angle2 * M_PI / 180),
          .y = sin(m_angle2 * M_PI / 180),
        };

        a2 = point_add(a1, a2);
        float target_rotation = atan2(a2.y, a2.x) * 180 / M_PI;
        NRF_LOG_RAW_INFO("[charger] TARGET ROTATION %d\n", (int) target_rotation);
        
        nav_motion_update(MOTION_ROTATE_ANGLE_PRECISE, (uint8_t*) &target_rotation, sizeof(float));

      break;

      case CHARGER_ENTRY_FORWARD1:
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY FORWARD 1\n");
  
        motors_hard_brake();
        led_rgb(0,1, 1);
        ir_set_mode(IR_DETECT_BOTH);
        ir_enable(false);

        // Set update parameters;
        current_intensity = 80;
        intensity_update = 20;
        max_val = 200;
    
        // Controlled forward
        nav_reset_beta();
        uint64_t time = 1000 * 1000 * 1000;
         
        uint8_t args[sizeof(uint64_t) + sizeof(int16_t)];
        ((uint64_t *)args)[0] = time;
        ((int16_t *)args)[4] = current_intensity;
        nav_motion_update(MOTION_FORWARD, args, sizeof(uint64_t) + sizeof(int16_t));  
      break;
      
      case CHARGER_ENTRY_FORWARD2:
        nav_end();
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY FORWARD 2\n");
  
        for(int i = 0; i < 4; i++){
          motors_rotate_cw(300);
          delayMs(100);
          motors_hard_brake();
          motors_rotate_ccw(300);
          delayMs(100);
          motors_hard_brake();
        }

        led_rgb(0, 0, 1);
        ir_set_mode(IR_DETECT_NONE);
        ir_enable(false);
    
        // Set update parameters;
        current_intensity = 80;
        intensity_update = 10;
        max_val = 120;

        m_second_count_internal = 4;

        motors_forward(current_intensity);
        motors_set(current_intensity, current_intensity);
      break;

      case CHARGER_ENTRY_DONE:
        NRF_LOG_RAW_INFO("[charger] CHARGER ENTRY DONE\n");

        if(ir_get_current_status() != IR_SENSORS_NONE){
          m_entry_state = CHARGER_ENTRY_DONE;
          ir_set_mode(IR_DETECT_NONE);
          ir_enable(false);
          motors_forward(150);
          m_second_count_internal = 4;
        }else{
          charger_next_stage();
        }
      break;

      default:
      break;
    }
    //NRF_LOG_RAW_INFO("[charger] Charger state updated to: %d\n", m_entry_state);
  }else{
    // HOLD
    NRF_LOG_RAW_INFO("[charger] HOLD\n");
    switch(m_entry_state){
      
      case CHARGER_ENTRY_ROTATION1:
        current_intensity = current_intensity + intensity_update;
        if(current_intensity > max_val) current_intensity = max_val;
        
        motors_rotate_cw(current_intensity);
      break;
      
      case CHARGER_ENTRY_ROTATION2:
        current_intensity = current_intensity + intensity_update;
        if(current_intensity > max_val) current_intensity = max_val;

        motors_rotate_ccw(current_intensity);
      break;

      case CHARGER_ENTRY_FORWARD1:
        current_intensity = current_intensity + intensity_update;
        if(current_intensity > max_val) current_intensity = max_val;
        
        nav_end();
        uint64_t time = 1000 * 1000 * 1000;
        uint8_t args[sizeof(uint64_t) + sizeof(int16_t)];
        ((uint64_t *)args)[0] = time;
        ((int16_t *)args)[4] = current_intensity;
        nav_motion_update(MOTION_FORWARD, args, sizeof(uint64_t) + sizeof(int16_t));
      break;
      
      case CHARGER_ENTRY_FORWARD2:
        current_intensity = current_intensity + intensity_update;
        if(current_intensity > max_val) current_intensity = max_val;
        
        motors_set(current_intensity, current_intensity);
        //motors_forward(current_intensity);
      break;

      default:
      break;
    }
  }
}

void charger_next_stage(void){
  charger_state_update(CHARGER_UPDATE_STATE_NEXT);
}

void charger_update_internal_count(void){
  //NRF_LOG_RAW_INFO("[charger] SECONDS %d\n", m_second_count_internal);
  if(m_charger_entry_enabled){
    NRF_LOG_RAW_INFO("[charger] internal: %d Current intensity %d, Increment %d, Max %d\n", m_second_count_internal, current_intensity, intensity_update, max_val);
    if(m_second_count_internal == 255){
      return;
    }
    m_second_count_internal--;
    if(m_second_count_internal == 0){
      m_second_count_internal = -1;
      charger_next_stage();
      //NRF_LOG_RAW_INFO("[charger] Evil line");
    }else{
      charger_state_update(CHARGER_UPDATE_STATE_HOLD);
    }
  }
}

void charger_set_internal_count(uint8_t val){
  m_second_count_internal = val;
}