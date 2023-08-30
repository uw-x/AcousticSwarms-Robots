#include "motors.h"
#include "nrf_drv_pwm.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "math.h"
#include "fuel_gauge.h"

#define MOTOR_ADJUSTMENT_DEBUG 0

#define MOTORS_TOP_VALUE (1000)
#define PWM_BASE_FREQ (1000000)
#define PWM_FREQ (PWM_BASE_FREQ / MOTORS_TOP_VALUE)

// Duration to wait before increasing motor speed (ms)
#define INCREMENT_INTERVAL (100)

// Amount to increase motor intensity by each INCREMENT_INTERVAL ms
#define MOTOR_INCREMENT_VAL (5)

// 1 more than the ceiling because last once will be repeated
#define NUM_TRANSIENT_REPEATS ( (PWM_FREQ / 1000) * INCREMENT_INTERVAL )
//#define NUM_TRANSIENT_REPEATS 10

#define min(a, b) (a < b ? a : b)
#define max(a, b) (a > b ? a : b)

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

static motor_t m1, m2;
static volatile bool motors_started;
static uint8_t common_control = MOTORS_DRV;

// For pulse mode
#define PULSE_INCREMENT
static uint64_t iter, pulse_period;
static uint16_t intensity;
static volatile bool pulsating, pulsedir;

// Seq 1 & Seq 2
static uint16_t duty_cycles[4][2];

static nrf_pwm_values_individual_t values_seq[2];

static int16_t val[64];

static nrf_pwm_sequence_t tseq1 = {
  .values.p_individual = &values_seq[0],
  .length = NRF_PWM_VALUES_LENGTH(values_seq[0]),
  .repeats = NUM_TRANSIENT_REPEATS,
  .end_delay = 0
};

static nrf_pwm_sequence_t tseq2 = {
  .values.p_individual = &values_seq[1],
  .length = NRF_PWM_VALUES_LENGTH(values_seq[1]),
  .repeats = NUM_TRANSIENT_REPEATS,
  .end_delay = 0
};

uint16_t _motors_speed_update(motor_t *m, uint8_t pin_id){
  if(m->current_speed[pin_id] < m->target_speed[pin_id]){
    m->current_speed[pin_id] = min(m->current_speed[pin_id] + MOTOR_INCREMENT_VAL, m->target_speed[pin_id]);
  }else if(m->current_speed[pin_id] > m->target_speed[pin_id]){
    m->current_speed[pin_id] = max(m->current_speed[pin_id] - MOTOR_INCREMENT_VAL, m->target_speed[pin_id]);
  }
  return m->current_speed[pin_id];
}

void _motors_all_speed_update(uint8_t idx){  
  values_seq[idx].channel_0 = _motors_speed_update(&m1, 0);
  values_seq[idx].channel_1 = _motors_speed_update(&m1, 1);
  values_seq[idx].channel_2 = _motors_speed_update(&m2, 0);
  values_seq[idx].channel_3 = _motors_speed_update(&m2, 1);
}

void motors_pwm_handler(nrfx_pwm_evt_type_t event_type){
  if(event_type == NRFX_PWM_EVT_END_SEQ0 || event_type == NRFX_PWM_EVT_END_SEQ1){
    uint8_t k = event_type == NRFX_PWM_EVT_END_SEQ1;
    
    if(pulsating){
      int i = iter % pulse_period;
      if(pulsedir){
        values_seq[k].channel_0 = val[i];
        values_seq[k].channel_1 = 0;
        values_seq[k].channel_2 = val[i];
        values_seq[k].channel_3 = 0;
      }else{
        values_seq[k].channel_1 = val[i];
        values_seq[k].channel_0 = 0;
        values_seq[k].channel_3 = val[i];
        values_seq[k].channel_2 = 0;
      }
      iter++;
    }else{
      _motors_all_speed_update(k);
    }

    nrf_pwm_values_t values;
    values.p_individual = &values_seq[k];
    nrfx_pwm_sequence_values_update(&m_pwm0, k, values);

    // TODO: when speed has stabalized, stop calling handler?
    // Re arm only when a speed modification is made

    //NRF_LOG_RAW_INFO("%08lld\n", systemTimeGetUs());
    //NRF_LOG_RAW_INFO("m1 current speed 1 %d\n", m1.current_speed[1]);
    //NRF_LOG_RAW_INFO("m2 current speed 0 %d\n", m2.current_speed[0]);
    //NRF_LOG_RAW_INFO("m2 current speed 1 %d\n", m2.current_speed[1]);
  }else if(NRFX_PWM_EVT_STOPPED){
    // Consider de-init-ing the PWM module for power saving?
    //NRF_LOG_RAW_INFO("STOPPED\n");
  }
}


bool motors_init(void){
  nrf_gpio_cfg_output(common_control);
  nrf_gpio_pin_clear(common_control);
  
  motors_started = false;

  // Channel # in PWM instance
  m1.p = 0;
  m1.n = 1;

  m2.p = 0;
  m2.n = 1;

  nrf_drv_pwm_config_t const config0 = {
      .output_pins =
      {
          MOTORS_M1I1 | NRF_DRV_PWM_PIN_INVERTED, // channel 0
          MOTORS_M1I2 | NRF_DRV_PWM_PIN_INVERTED, // channel 1
          MOTORS_M2I1 | NRF_DRV_PWM_PIN_INVERTED, // channel 2
          MOTORS_M2I2 | NRF_DRV_PWM_PIN_INVERTED// channel 3
      },
      .irq_priority = APP_IRQ_PRIORITY_LOWEST,
      .base_clock   = NRF_PWM_CLK_8MHz,
      .count_mode   = NRF_PWM_MODE_UP,
      .top_value    = MOTORS_TOP_VALUE , // 8M/1e3 = 8kHz 
      .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
      //.load_mode    = NRF_PWM_LOAD_COMMON,
      .step_mode    = NRF_PWM_STEP_AUTO
  };
  ret_code_t ret = nrf_drv_pwm_init(&m_pwm0, &config0, motors_pwm_handler); 
  APP_ERROR_CHECK(ret);

  return ret == NRF_SUCCESS;
}


// Linear up ramp followed by down ramp on motors
// minimum stength, max strength, period (in 100 ms intervals)
void _motors_pulse(uint16_t minval, uint16_t maxval, uint16_t period, bool forward){
  //motors_stop();

  if(!motors_started) motors_start();
  
  iter = 0;
  pulsating = true;
  pulsedir = forward;
  val[0] = minval;

  pulse_period = period;

  int sign = 1;
  int16_t increment = 2 * (maxval - minval) / period;
  for(int i = 1; i < 2 * period; i++){
    val[i] = val[i-1] + increment * sign;
    if(val[i] > maxval){
      val[i] = maxval - (val[i] - maxval);
      sign = -1;
    }
  }
  
}

void motors_pulse_forward(uint16_t minval, uint16_t maxval, uint16_t period){
  _motors_pulse(minval, maxval, period, 1);
}

void motors_pulse_backward(uint16_t minval, uint16_t maxval, uint16_t period){
  _motors_pulse(minval, maxval, period, 0);
}


bool motors_deinit(void){
  nrf_gpio_pin_clear(common_control);
  nrf_drv_pwm_uninit(&m_pwm0);
  return true;
}

void _motor_drive(motor_t *m, int16_t speed){
  speed = (int16_t) (fg_get_motor_compensation() * speed);
  if (speed > 0){
    m->target_speed[m->p] = speed;
    m->target_speed[m->n] = 0;
  }else{
    m->target_speed[m->p] = 0;
    m->target_speed[m->n] = -speed;
  }
}

void _motor_drive_direct(motor_t *m, int16_t speed){
  speed = (int16_t) (fg_get_motor_compensation() * speed);
  if (speed > 0){
    m->target_speed[m->p] = speed;
    m->target_speed[m->n] = 0;
    m->current_speed[m->p] = speed;
    m->current_speed[m->n] = 0;
  }else{
    m->target_speed[m->p] = 0;
    m->target_speed[m->n] = -speed;
    m->current_speed[m->p] = 0;
    m->current_speed[m->n] = -speed;
  }
}

void _motor_brake(motor_t *m){
  m->target_speed[m->p] = 0;
  m->target_speed[m->n] = 0;

  //m->current_speed[m->p] = 0;
  //m->current_speed[m->n] = 0;
}

void _motor_hard_brake(motor_t* m){
  m->target_speed[m->p] = 0;
  m->target_speed[m->n] = 0;

  m->current_speed[m->p] = 0;
  m->current_speed[m->n] = 0;
}

void _motor_loose(motor_t *m){
  m->target_speed[m->p] = 0;
  m->target_speed[m->n] = 0;
}

void _motors_pwm_start(){
  for(int i = 0; i < 2; i++){
    values_seq[i].channel_0 = 0;
    values_seq[i].channel_1 = 0;
    values_seq[i].channel_2 = 0;
    values_seq[i].channel_3 = 0;
  }

  //(void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
  (void)nrf_drv_pwm_complex_playback(&m_pwm0, &tseq1, &tseq2, 1, 
                                     NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ0 | 
                                     NRF_DRV_PWM_FLAG_SIGNAL_END_SEQ1 |
                                     NRF_DRV_PWM_FLAG_LOOP);
}


void motors_start(void){
  motors_started = true;
  
  _motors_pwm_start();
  nrf_gpio_pin_set(common_control);
}

void motors_stop(void){
  if(motors_started){
    nrf_gpio_pin_clear(common_control);
    nrfx_pwm_stop(&m_pwm0, false);
  
    motors_started = false;
    pulsating = false;
  }
}

void motors_set(int16_t left, int16_t right){
  if(!motors_started) motors_start();
  _motor_drive(&m2, left);
  _motor_drive(&m1, right);
}

void motors_forward(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive(&m1, speed);
  _motor_drive(&m2, speed);
}

void motors_backward(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive(&m1, -speed);
  _motor_drive(&m2, -speed);
}

void motors_rotate_ccw(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive(&m1, -speed);
  _motor_drive(&m2, speed);
}

void motors_rotate_cw(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive(&m1, speed);
  _motor_drive(&m2, -speed);
}

void motors_rotate_ccw_direct(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive_direct(&m1, -speed);
  _motor_drive_direct(&m2, speed);
}

void motors_rotate_cw_direct(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive_direct(&m1, speed);
  _motor_drive_direct(&m2, -speed);
}

void motors_left(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();
  
  _motor_drive(&m1, speed);
  _motor_loose(&m2);
}

void motors_right(uint16_t speed){
  pulsating = false;
  if(!motors_started) motors_start();

  _motor_drive(&m2, speed);
  _motor_loose(&m1);
}

void motors_brake(){
  pulsating = false;
  _motor_brake(&m1);
  _motor_brake(&m2);
}

void motors_hard_brake(){
  pulsating = false;
  _motor_hard_brake(&m1);
  _motor_hard_brake(&m2);
}

void motors_loose(){
  pulsating = false;
  _motor_loose(&m1);
  _motor_loose(&m2);
}

int16_t _clamp(int16_t val, int16_t lower, int16_t upper){
  if(val < lower) return lower;
  else if(val > upper) return upper;
  return val; 
}

#define ANGLE_CORR (5 * 3.141593 / 180) // 5 degree correction in velocity space

void motors_adjust(float speed, float beta){
  pulsating = false;
  if(!motors_started) motors_start();
  
  int16_t sign = 1;
  if(speed < 0){
    sign = -1;
    speed = -speed;
  }

  int16_t m1_speed = round(speed * (1 - beta) * 2);
  int16_t m2_speed = round(speed * beta * 2);

  m1_speed = _clamp(m1_speed, 0, 2 * speed);
  m2_speed = _clamp(m2_speed, 0, 2 * speed);
  
  _motor_drive(&m1, sign * m1_speed);
  _motor_drive(&m2, sign * m2_speed);

#if MOTOR_ADJUSTMENT_DEBUG
  NRF_LOG_RAW_INFO("Velocities: (%d %d)\n", m1.target_speed[m1.p], m2.target_speed[m2.p]);
#endif
}

//TODO: Figure this out
void motors_test(){
  motors_start();
  
  //NRF_LOG_RAW_INFO("FORWARD\n");
  motors_forward(20);
  //nrf_delay_ms(T1);
}