// MAKE SURE ROBOT NAME IS SHIO NOT SHIOG

/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

 
#include <assert.h>

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"

#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "app_util_platform.h"

#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "nrf_egu.h"

#include "gpio.h"
#include "ble_manager.h"
#include "event.h"
#include "timers.h"
#include "ble_config.h"
#include "time_sync.h"
#include "audio.h"

#include "led.h"
#include "bsp_btn_ble.h"
#include "i2c.h"
#include "i2s.h"
#include "buzzer.h"
#include "fuel_gauge.h"
#include "accel.h"
#include "motors.h"
#include "recorder.h"
#include "ts_tasks.h"
#include "charger.h"
#include "nav.h"
#include "point.h"
#include "gyro.h"
#include "channel_utils.h"
#include "ir.h"
#include "ble_dcs.h"
#include "profile.h"


// In case something is going wrong, check on both receiver & robots:
// Always use stack allocation for ringbuffers!!
// Always compile as release!!

APP_TIMER_DEF(resetTimer);

static uint8_t micData[PACKET_LENGTH];
static bool bleRetry = false;

static bool m_pdm_started = false;
static bool m_stream_started = false;
static uint32_t m_interval = 0;

static bool m_audio_data_requested = false;

// Enough for 250 ms @ 50kHz (as int16)
//#define RECORDING_BUFFER_SIZE 35000

// Enough for 50 ms @ 62.5kHz (as int16)
#define RECORDING_BUFFER_SIZE (6500 * 2)
//#define RECORDING_BUFFER_SIZE 8
static int16_t rec_single_channel_buffer[3126];

#define SHIELD_ENABLED 1

static uint8_t m_record_buffer[RECORDING_BUFFER_SIZE] = {0};
static recording_t s_recording;

static uint64_t expectedBufferCount = 0;

static int64_t m_audio_buffer_samples = 0;

static bool m_sensor_notifications_enabled = false;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_RAW_INFO("DEADBEEF: SOFTDEVICE INTERNAL ERRROR IN %s AT LINE %d\n", p_file_name, line_num);
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = led_indicate(LED_OFF);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

void powerEnterSleepMode(void)
{
  ret_code_t err_code;

  NRF_LOG_RAW_INFO("%08d [power] powering off...\n", systemTimeGetMs());

  err_code = led_indicate(LED_OFF);
  APP_ERROR_CHECK(err_code);
  
  motors_deinit();
  accel_deinit();
  
#if SHIELD_ENABLED
  gyro_deinit();
#endif
  
  // Don't ?
  //fg_deinit();
  
  i2c_deinit();
  i2s_deinit();
  audioDeInit();

  led_deinit();
  ir_disable();

  ////spiDeInit();
  //delayMs(1);

  NRF_LOG_RAW_INFO("%08d [power] Shut down.\n", systemTimeGetMs());
  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void){
    if (NRF_LOG_PROCESS() == false){
        nrf_pwr_mgmt_run();
    }
}

static void resetTimerCallback(void * p_context){
  NVIC_SystemReset();
}

void TIMER3_IRQHandler(){
  if(NRF_TIMER3->EVENTS_COMPARE[4]){
    if(!ts_sync_tasks_started()){
      ts_sync_tasks_start(true);
      NRF_LOG_RAW_INFO("%08d [main] TIME SYNC MASTER\n", systemTimeGetMs());
    }else{
      NRF_LOG_INFO("Disabling PPIs");
      ts_sync_tasks_disable_pending_ppis();
    }
    NRF_TIMER3->EVENTS_COMPARE[4] = 0;
    // Disable interrupt (this is supposed to be a one time thing)
    NRF_TIMER3->INTENCLR = TIMER_INTENSET_COMPARE4_Set << TIMER_INTENSET_COMPARE4_Pos;
  }
  //NRF_LOG_INFO("Unhandled TIMER3 Interrupt");
}

static void timeSyncInit(void)
{
  uint32_t       err_code;
  uint8_t        rf_address[5] = {0xDE, 0xAD, 0xBE, 0xEF, 0x19};
  ts_params_t    ts_params;

  ts_params.high_freq_timer[0] = NRF_TIMER3;
  ts_params.high_freq_timer[1] = NRF_TIMER2;
  ts_params.rtc             = NRF_RTC1;
  ts_params.egu             = NRF_EGU3;
  ts_params.egu_irq_type    = SWI3_EGU3_IRQn;
  ts_params.ppi_chg         = 0;
  ts_params.ppi_chns[0]     = 1;
  ts_params.ppi_chns[1]     = 2;
  ts_params.ppi_chns[2]     = 3;
  ts_params.ppi_chns[3]     = 4;
  ts_params.rf_chn          = 125; /* For testing purposes */
  memcpy(ts_params.rf_addr, rf_address, sizeof(rf_address));

  err_code = ts_init(&ts_params);
  APP_ERROR_CHECK(err_code);
  
  err_code = ts_enable();
  APP_ERROR_CHECK(err_code);
}

void timestamp_capture_init(void){
  nrf_ppi_channel_endpoint_setup(
    NRF_PPI_CHANNEL6,
    (uint32_t) nrf_egu_event_address_get(NRF_EGU0, NRF_EGU_EVENT_TRIGGERED0),
    (uint32_t) nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_CAPTURE3));

  nrf_ppi_fork_endpoint_setup(
    NRF_PPI_CHANNEL6,
    (uint32_t) nrf_timer_task_address_get(NRF_TIMER3, NRF_TIMER_TASK_CAPTURE3)); 
  
  nrf_ppi_channel_enable(NRF_PPI_CHANNEL6);
}

void processQueue(void);

bool system_init(bool init_bt){
  bool ok = true;
  
  log_init(); // Causes very strong noise for some platforms??
  profile_init();
  timersInit();
  gpioInit();
  power_management_init();
  eventQueueInit();

  led_init();

  ok = ok && i2c_init();
  ok = ok && fg_init();
  
  #if SHIELD_ENABLED
  //mag_init();
  gyro_init();
  #endif
  
  ok = ok && accel_init();
  
  ok = ok && buzzer_init();
  ok = ok && motors_init();
  charger_init();

  ret_code_t err_code;
  err_code = app_timer_create(&resetTimer, APP_TIMER_MODE_SINGLE_SHOT, resetTimerCallback);
  APP_ERROR_CHECK(err_code);

  //buttons_leds_init(&erase_bonds);
  led_init();

  audioInit();

  if(init_bt){
    bleInit();
  
    timeSyncInit();
    timestamp_capture_init();
  }  
  // Initializes all time synchronized tasks
  ts_sync_tasks_init();

  // Initializes chirp offset detection for localization
  init_chirp_fft();

  nav_init();

  ok = ok && ir_init();
  APP_ERROR_CHECK_BOOL(ok);

  // Get fuel guage readings
  fg_update_current_measurements();
  
  // Wait for sensor calibration
#if SHIELD_ENABLED
  while(!gyro_ready()){
    idle_state_handle();
    processQueue();
  }
#endif
  
  while(!accel_ready()){
    idle_state_handle();
    processQueue();
  }

  return ok;
}

void print_sensor_info(){
  float battery_level = fg_get_current_battery_level();
  float soc = fg_get_current_soc();
  NRF_LOG_INFO("Current voltage: " NRF_LOG_FLOAT_MARKER "V\nCurrent state of charge: "
              NRF_LOG_FLOAT_MARKER"\n",
              NRF_LOG_FLOAT(battery_level),
              NRF_LOG_FLOAT(soc));
}

int task_time = 3;

#include "math.h"
#include "arm_math.h"
float interp_table[1000 + 5];

// Resample audio from S1 samples to S2 samples
void audio_resample(int16_t *buffer, uint16_t S1, uint16_t S2, int channels){
  if(S1 == S2) return;
  float step = 1.0 * S1 / S2;

  arm_q15_to_float(buffer, interp_table, S1);

  //NRF_LOG_RAW_INFO("Convert: %lld us\n", t2 - t1);

  arm_linear_interp_instance_f32 S = {S1, 0, 1, interp_table};
  float val = 0;
  
  // We only care about every m-th element
  for(int i = 0; i < S2; i++){
    buffer[i] = INT16_MAX * arm_linear_interp_f32(&S, val);
    val += step;
  }
}

/**@brief Function for application main entry.
 */
int main(void){
    // Initialize.
    bool init_bt = true;
    //bool init_bt = false;
    
    system_init(init_bt);
    print_sensor_info();

    bleAdvertisingStart();
    //nav_begin();

    //accel_enable_collision_detection(true);

    //motors_deinit();
    //accel_deinit();
  
    //#if SHIELD_ENABLED
    //  gyro_deinit();
    //#endif
  
    ////i2c_deinit();
    //i2s_deinit();
    ////audioDeInit();

    //led_deinit();
    //ir_disable();

    //accel_enable_collision_detection(true);
    //charger_start_station_entry();

    //nav_forward(5e6, 200, 1);
    //////delayMs(1005);
    //nav_rotate_angle(180);
    //////delayMs(500);
    //nav_rotate_angle_precise(90);
    //////delayMs(500);
    //nav_backward(1e6, 200, 1);
    ////delayMs(1005);
    //nav_rotate_time(2e6, 200);

    
    //ir_set_mode(IR_MODE_COUNTER);
    //ir_set_target_depth(100);
    //motors_forward(150);
    //ir_enable(false);

    //ir_set_mode(IR_MODE_EDGE_DETECTION);

    //uint8_t args[sizeof(uint64_t) + sizeof(int16_t)];
    //uint64_t time = 1e9;
    //((uint64_t *)args)[0] = time;
    //((int16_t *)args)[4] = 250;
    //nav_motion_update(MOTION_FORWARD, args, sizeof(uint64_t) + sizeof(uint16_t));

    //ir_enable(false);

    //nav_begin();

    for (;;){
        idle_state_handle();
        processQueue();
    }
}


void processQueue(void){
  if (!eventQueueEmpty()) {
    //NRF_LOG_RAW_INFO("%08d [main] Received event:%d\n", systemTimeGetUs(), eventQueueFront());
    switch(eventQueueFront()) {
      case EVENT_BLE_SEND_METADATA:
        ts_sync_task_send_metadata();
       break;

      case EVENT_AUDIO_MIC_DATA_READY:{
        //NRF_LOG_RAW_INFO("[audio] Audio data ready\n");
        uint8_t *buffer;
        uint8_t data_length;
        uint8_t fetched = 0;
        while(data_length = audioGetMicData(&buffer)){
          // Set audioStreamStarted to true
          fetched++;
          
          memcpy_fast(micData, buffer, PACKET_LENGTH);
          //if(tmp){
          //  for(int i = 0 ; i < 50 /*data_length/2*/; i++){
          //    NRF_LOG_RAW_INFO("%d ", (micData[2 * i + 1] << 8) + micData[2 * i]);
          //  }NRF_LOG_RAW_INFO("\n");
          //  tmp = false;
          //}
          // Sometimes a PDM buffer gets thrown into the ether?
          //uint32_t actualBufferCount = audioGetPdmBufferCount();
          //if (++expectedBufferCount != actualBufferCount) {
          //  //NRF_LOG_RAW_INFO("%08d [audio] expected(%d) != actual(%d)\n", systemTimeGetMs(), expectedBufferCount, actualBufferCount);
          //  for (int i = 0; i < (actualBufferCount - expectedBufferCount); i++) { blePushSequenceNumber(); }
          //  expectedBufferCount = actualBufferCount;
          //}          
          //NRF_LOG_RAW_INFO("%08d [ble] Length %d\n", systemTimeGetMs(), data_length);
          if(audio_get_mode() == AUDIO_MODE_STREAMING){
            assert(audioStreamStarted());
            if (bleBufferHasSpace(data_length)) {
              //NRF_LOG_RAW_INFO("%08d [ble] written %d samples to queue\n", systemTimeGetMs(), PACKET_LENGTH);
              bleSendData((uint8_t *) micData, data_length);
              //NRF_LOG_RAW_INFO("%08d [ble] buffer available bytes: %d\n", systemTimeGetMs(), bleGetRingBufferBytesAvailable());
              //NRF_LOG_RAW_INFO("%08d [ble] %d %d %d\n", 
              //          (micData[0] << 8) + micData[1],
              //          (micData[2] << 8) + micData[3],
              //          (micData[4] << 8) + micData[5]);
            } else {
              // No space, drop this data
              NRF_LOG_RAW_INFO("%08d [ble] dropped %d bytes\n", systemTimeGetMs(), data_length);
              blePushSequenceNumber();
            }
          }else if(audio_get_mode() == AUDIO_MODE_RECORDING){
            uint8_t *adjusted_ptr = micData;
            uint16_t adjusted_length;
            //NRF_LOG_RAW_INFO("%08d [main] Record\n", systemTimeGetMs());
            //NRF_LOG_RAW_INFO("%08d [main] m_audio_buffer_samples: %d\n", systemTimeGetMs(), m_audio_buffer_samples);
            // If recording is expected during this time
            if(save_recording(m_audio_buffer_samples, data_length, &adjusted_ptr, &adjusted_length)){              
              //NRF_LOG_RAW_INFO("%08d [main] Saving\n", systemTimeGetMs());
              //if(adjusted_length < 200)
              //  NRF_LOG_RAW_INFO("Storing %lld bytes (%lld samples)\n", adjusted_length, adjusted_length/2);
              // Save buffer
              if(s_recording.length == 0){
                s_recording.start_time = systemTimeGetUs();
              }
              recorder_add(&s_recording, adjusted_ptr, adjusted_length);
            }else if(audio_recording_finished(m_audio_buffer_samples)){
              //if(recorder_available(&s_recording)){
              //  //NRF_LOG_RAW_INFO("%08d Sending\n", systemTimeGetUs());
              //  recorder_send_all(&s_recording);
              //}else{
                
              //}
              //
              //NRF_LOG_RAW_INFO("%08d [main] Recorder length: %d\n", systemTimeGetMs(), s_recording.length);
              //ble_notify_recording_finished();
              eventQueuePush(EVENT_RECORDING_FINISHED);
            }
            m_audio_buffer_samples += data_length/2;
          }
        }
        break;
      }
    
      case EVENT_RECORDING_PROCESS_FINISHED:{
        bool flag = true;
        for(int i = 911; i < 915; i++){
          if(((int16_t*) s_recording.data)[i] != 0) flag = false;
        }
        if(flag) {
          NRF_LOG_RAW_INFO("%08d [main] ERROR\n", systemTimeGetUs(), s_recording.length);
          //APP_ERROR_CHECK_BOOL(0);
        }else{
          NRF_LOG_RAW_INFO("%08d [main] SAFE\n", systemTimeGetUs(), s_recording.length);
        }
        if(recorder_available(&s_recording) && m_audio_data_requested){
          NRF_LOG_RAW_INFO("%08d Sending %d samples\n", systemTimeGetUs(), s_recording.length);          
          APP_ERROR_CHECK_BOOL(audio_get_channels() * 3126 == s_recording.length);
          s_recording.requested = true;
          recorder_send_all(&s_recording);
        }
        }
        break;
      
      case EVENT_RECORDING_SEND_FINISHED:
        // Do nothing for now.
      break;

      case EVENT_RECORDING_FINISHED:
        if(ble_cus_indication_in_progress()){
          NRF_LOG_INFO("[ble_indicate_status]  ***** INDICATION IN PROGRESS *****");
        }
  
        if(nav_xcorr_stream_enabled()){
          // Begin chirp offset computation:
          
          if(audio_get_channels() == 2){
            // Get left channel
            recorder_deinterleave_channel(&s_recording, rec_single_channel_buffer, 0);
          
            uint64_t t1 = systemTimeGetUs();
            //uint16_t left_offset = detection_phase_left(rec_single_channel_buffer, s_recording.length/2);
            detection_phase_left(rec_single_channel_buffer, s_recording.length/2);
            uint64_t t2 = systemTimeGetUs();
            
            NRF_LOG_RAW_INFO("%08d [main] XCORR LEFT CHANNEL TIME: %lld\n", systemTimeGetMs(), t2 - t1);
            eventQueuePush(EVENT_CHIRP_DETECION_PHASE_RIGHT);
          }else{
            eventQueuePush(EVENT_CHIRP_DETECION_PHASE_FINAL);
          }
        }else{
          eventQueuePush(EVENT_RECORDING_PROCESS_FINISHED);
        }
      break;

      case EVENT_CHIRP_DETECION_PHASE_RIGHT:
        // Get right channel
        recorder_deinterleave_channel(&s_recording, rec_single_channel_buffer, 1);
      
        uint64_t t1 = systemTimeGetUs();
        //uint16_t left_offset = detection_phase_right(rec_single_channel_buffer, s_recording.length/2);
        detection_phase_right(rec_single_channel_buffer, s_recording.length/2);
        uint64_t t2 = systemTimeGetUs();
        NRF_LOG_RAW_INFO("%08d [main] XCORR RIGHT CHANNEL TIME: %lld\n", systemTimeGetMs(), t2 - t1);
        eventQueuePush(EVENT_CHIRP_DETECION_PHASE_FINAL);
      break;

      case EVENT_CHIRP_DETECION_PHASE_FINAL:
        if(nav_xcorr_stream_enabled()){
          uint16_t sample_offset;
          float32_t avg_offset;

          uint64_t t1 = systemTimeGetUs();
          //detection_phase2(&sample_offset, &avg_offset, ts_master());
          sample_offset = detection_phase_combine(ts_master(), 1);
          uint64_t t2 = systemTimeGetUs();

          NRF_LOG_RAW_INFO("%08d [main] COMBINE TIME: %lld\n", systemTimeGetMs(), t2 - t1);

          s_recording.end_time = systemTimeGetUs();

          nav_send_all(sample_offset, &s_recording);

          eventQueuePush(EVENT_RECORDING_PROCESS_FINISHED);
        }
      break;

      case EVENT_RECORDER_RESET:
        recorder_reset(&s_recording);
      break;

      case EVENT_AUDIO_START_CLICK:
        NRF_LOG_RAW_INFO("%08d [main] CHIRP\n", systemTimeGetMs())
        buzzer_play_chirp(1);
        // Chirp at 0 + 40 * 50ms = 2000ms
        //ts_sync_task_add(TS_TASK_SPK_START, MASTER_OVERFLOW_TRIGGER_COUNT + 40);
      break;

      case EVENT_AUDIO_MIC_NOTIFICATIONS_ENABLED:
        NRF_LOG_RAW_INFO("%08d [main] ENABLED AUDIO NOTIFICATIONS\n", systemTimeGetMs())
        m_audio_data_requested = true;
        resetBLE();
      break;

      case EVENT_AUDIO_MIC_NOTIFICATIONS_DISABLED:
        m_audio_data_requested = false;
        resetBLE();
        audioDeInit();
      break;

      case EVENT_SENSOR_NOTIFICATIONS_ENABLED:
        m_sensor_notifications_enabled = true;
      break;

      case EVENT_SENSOR_NOTIFICATIONS_DISABLED:
        m_sensor_notifications_enabled = false;
      break;

      case EVENT_BLE_DATA_STREAM_START:
        NRF_LOG_RAW_INFO("%08d [main] Streaming enabled\n", systemTimeGetMs());
        audio_set_mode(AUDIO_MODE_STREAMING);
        audio_ready();
        break;

      case EVENT_BLE_DATA_RECORD_START:
        m_audio_buffer_samples = 0;
        NRF_LOG_RAW_INFO("%08d [main] Recording enabled\n", systemTimeGetMs());
        audio_set_mode(AUDIO_MODE_RECORDING);
        reset_tracking();
        recorder_init(&s_recording, m_record_buffer, RECORDING_BUFFER_SIZE);
        audio_ready();
        break;

      case EVENT_BLE_DATA_STREAM_STOP:{
        // NVIC_SystemReset();
        // app_timer_start(resetTimer, APP_TIMER_TICKS(2000), NULL);
        audio_set_mode(AUDIO_MODE_NONE);
        resetBLE();
        break;
      }
      
      case EVENT_BLE_SEND_DATA_DONE:
        // BLE just finished, attempt to fill in more data
        //NRF_LOG_RAW_INFO("%08d [ble] [%d] (delayed) sent %d samples\n", systemTimeGetMs(), getSequenceNumber(), PDM_DECIMATION_BUFFER_LENGTH);
        //send();
        //NRF_LOG_RAW_INFO("%08d [ble] buffer available bytes: %d\n", systemTimeGetMs(), bleGetRingBufferBytesAvailable());
        //if(audio_get_mode() == AUDIO_MODE_STREAMING){
        //  send();
        //}else if(audio_get_mode() == AUDIO_MODE_RECORDING){
        //  if(s_recording.requested && recorder_available(&s_recording)){
        //    recorder_send_all(&s_recording);
        //  }
        //  //if(nav_xcorr_stream_enabled() && ble_xcorr_available()){
        //  //  ble_send_status_update_data();
        //  //}
        //}

        if(audio_get_mode() == AUDIO_MODE_STREAMING && ble_audio_stream_buffer_available()){
          send();
        }else if(audio_get_mode() == AUDIO_MODE_RECORDING &&
                 s_recording.requested &&
                 recorder_available(&s_recording)){
          recorder_send_all(&s_recording);
        }else if(ble_status_notification_buffer_available()){
          ble_notification_buffer_process();
        }else if(ble_status_indication_buffer_available()){
          ble_indication_buffer_process();
        }

        break;

      case EVENT_BLE_IDLE:
        powerEnterSleepMode();
        break;

      case EVENT_BLE_DISCONNECTED:
        NRF_LOG_RAW_INFO("Disconnected\n");
        //NRF_TIMER3->EVENTS_COMPARE[4] = 0; // What is this doing??
        NVIC_SystemReset();
        break;

      case EVENT_TIMESYNC_MASTER_ENABLE:
        ts_tasks_set_role(TS_ROLE_MASTER);
        break;

      case EVENT_TIMESYNC_SLAVE_ENABLE:
        ts_tasks_set_role(TS_ROLE_SLAVE);
      break;

      case EVENT_TIMESYNC_DISABLE:
        ts_tasks_set_role(TS_ROLE_OFF);
      break;

      case EVENT_TIMESYNC_PACKET_SENT:
        if(ts_enabled()){
          assert(ts_master());
          //NRF_LOG_RAW_INFO("%08d [main] SENT TIMESYNC PACKET\n", systemTimeGetUs()
          m_interval = num_master_overflows();
          ts_tasks_process(m_interval);
        }
      break;

      case EVENT_TIMESYNC_PACKET_RECEIVED:
        if(ts_enabled()){
          //NRF_LOG_RAW_INFO("%08d [main] RECEIVED TIMESYNC PACKET\n", systemTimeGetUs());

          //uint64_t t1 = systemTimeGetUs();
          CRITICAL_REGION_ENTER();
          assert(!ts_master());
          //NRF_LOG_RAW_INFO("[BEFORE] Timer3 counterval %07d\n", NRF_TIMER3->CC[3]);

          //delayMs(3);
        
          m_interval = num_master_overflows();

          if(!ts_sync_tasks_started()){
            ts_sync_tasks_start(false);
            NRF_LOG_RAW_INFO("%08d [main] TIME SYNC SLAVE\n", systemTimeGetUs());
          }
          
          if(ts_tasks_process(m_interval)){
            //NRF_LOG_RAW_INFO("Timer3 counterval %07d\n", NRF_TIMER3->CC[3]);
          }

          //uint64_t t2 = systemTimeGetUs();

          //NRF_LOG_RAW_INFO("Time taken %08d\n", t2 - t1);
          CRITICAL_REGION_EXIT();


          //audioUpdateTicksAhead();
          // TODO: Update speaker ticks as well.
        }
        break;

      case EVENT_CHARGER_CONNECTED:
        NRF_LOG_RAW_INFO("%08d [main] Charging started\n", systemTimeGetUs());
      break;

      case EVENT_CHARGER_DISCONNECTED:
        NRF_LOG_RAW_INFO("%08d [main] Charging stopped\n", systemTimeGetUs());
      break;  

      case EVENT_TIMERS_ONE_SECOND_ELAPSED:{
        //print_sensor_info();
        //buzzer_play_chirp(1);
        //if(--task_time == 0){
        //  uint8_t args = 200;
        //  nav_motion_update(MOTION_FORWARD, &args, 1);
        //}
        
        //if(nav_get_current_action().type != NAV_ACTION_IDLE){
        //  nav_state_t s1, s2;
        //  s1 = nav_get_current_state();
        //  ble_add_xcorr_data(0, &s1, &s2);
        //  NRF_LOG_RAW_INFO("%08d [main] SENDING XCOR\n", systemTimeGetUs());
        //}
        //nav_rotate(30); // Rotate by 30 degrees.
        //ir_sensors_status_t status = ir_get_current_status();
        //NRF_LOG_RAW_INFO("%08d [main] ONE SECOND\n", systemTimeGetUs());
        //ir_sensors_status_t status = ir_get_current_status();
        //NRF_LOG_RAW_INFO("%08d [main] IR STATUS %d\n", systemTimeGetUs(), status);
        //NRF_LOG_RAW_INFO("%08d [main] ir enabled %d\n", systemTimeGetUs(), ir_enabled());

        //NRF_LOG_RAW_INFO("Accel interrupt state %d\n", nrf_gpio_pin_read(ACCEL_INT_PIN));

        //NRF_LOG_RAW_INFO("%08d [main] ir mode %d\n", systemTimeGetUs(), ir_get_mode());
        
        charger_update_internal_count();
        break;
      }
      case EVENT_FUEL_GAUGE_INTERRUPT:
        fg_handle_interrupt();
        if(fg_stream_enabled()){
          bleSendSensorData(fg_get_sensor_class());
        }
        break;

      case EVENT_ACCEL_INTERRUPT:
        //NRF_LOG_RAW_INFO("%08d [main] ACCEL INTERRUPT\n", systemTimeGetUs());
        accel_handle_interrupt();
        if(accel_ready()){
          if(accel_stream_enabled()){
            bleSendSensorData(accel_get_sensor_class());
          }        
          nav_acc_update();
        }
        break;

      case EVENT_ENABLE_MOVING_COLLISION_DETECTION:
        accel_enable_collision_detection(true);
      break;

      case EVENT_ENABLE_STATIONARY_COLLISION_DETECTION:
        accel_enable_collision_detection(false);
      break;

      case EVENT_DISABLE_COLLISION_DETECTION:
        accel_disable_collision_detection();
      break;
      
      //case EVENT_MAG_INTERRUPT:
      //  mag_handle_interrupt();
      //  if(mag_stream_enabled()){
      //    NRF_LOG_RAW_INFO("[main] Sending mag data\n");
      //    bleSendSensorData(mag_get_sensor_class());
      //  }
      //  //NRF_LOG_RAW_INFO("[main] Heading: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(mag_get_heading()));
      //break;

      case EVENT_GYRO_INTERRUPT:
        gyro_handle_interrupt();
        if(gyro_ready()){
          //nav_heading_update();
          nav_gyro_update();
          if(gyro_stream_enabled()){
            //NRF_LOG_RAW_INFO("[main] Sending gyro data\n");
            bleSendSensorData(gyro_get_sensor_class());
            //nav_send_all(0);
          }
        }
        //NRF_LOG_RAW_INFO("[main] Heading: " NRF_LOG_FLOAT_MARKER "\n", NRF_LOG_FLOAT(mag_get_heading()));
      break;

      case EVENT_MOTION_CORRECTION:{
        //nav_state_t s1;
        //s1 = nav_get_current_state();
        //ble_indicate_motion_correction(&s1);
      }
      break;

      case EVENT_PING: {
        uint8_t byte = 0xff;
        NRF_LOG_RAW_INFO("%08d PINGING\n", systemTimeGetMs());
        ble_indicate_status(DCS_STATUS_PING, &byte, 1);
      }
      break;

      case EVENT_IR_STATUS_UPDATED:{
        ir_sensors_status_t status = ir_get_current_status();
        NRF_LOG_RAW_INFO("%08d IR SATUS: %d\n", systemTimeGetMs(), status);

        //switch(status){
        //  case IR_SENSORS_BOTH:
        //    led_rgb(0,0,1);
        //  break;
        //  case IR_SENSORS_RIGHT:
        //    led_rgb(0,1,1);
        //  break;
        //  case IR_SENSORS_LEFT:
        //    led_rgb(1,1,0);
        //  break;
        //  case IR_SENSORS_NONE:
        //    led_rgb(0,1,0);
        //  break;
        //}

        if(ir_get_mode() == IR_MODE_EDGE_DETECTION){        
          switch(status){
            case IR_SENSORS_BOTH:
            break;
            default:
              nav_recover();
            break;
          }
        }else if(ir_get_mode() == IR_MODE_COUNTER){
          if(ir_get_previous_status() == IR_SENSORS_BOTH && 
             (status == IR_SENSORS_LEFT || status == IR_SENSORS_RIGHT) ){
            ir_increment_marker_count();
            NRF_LOG_RAW_INFO("MARKER FOUND\n");
            if(ir_target_depth_reached()){
              //motors_brake();
              motors_hard_brake();
              nav_end();
              ir_disable();
            }
          }else if(status == IR_SENSORS_NONE){
            // Reached the final marker.
            NRF_LOG_RAW_INFO("FOUND LAST MARKER\n");
            motors_hard_brake();
            nav_end();
            ir_disable();
          }
        }else if(ir_get_mode() == IR_DETECT_LEFT){
          if(status == IR_SENSORS_LEFT){
            motors_hard_brake();
            ir_disable();
            //charger_set_internal_count(1);
            charger_next_stage();
          }
        }else if(ir_get_mode() == IR_DETECT_RIGHT){
          if(status == IR_SENSORS_RIGHT){
            motors_hard_brake();
            ir_disable();
            //charger_set_internal_count(1);
            charger_next_stage();
          }
        }else if(ir_get_mode() == IR_DETECT_BOTH){
          if(status == IR_SENSORS_BOTH){
            motors_hard_brake();
            ir_disable();
            //charger_set_internal_count(1);
            charger_next_stage();
          }
        }else if(ir_get_mode() == IR_DETECT_NONE){
          if(status == IR_SENSORS_NONE){
            motors_hard_brake();
            ir_disable();
            //NRF_LOG_RAW_INFO("%08d [main] IR DETEC TNONE\n", systemTimeGetMs());
            //charger_set_internal_count(1);
            charger_next_stage();
          }
        }else if(ir_get_mode() == IR_MODE_CHARGING_STATION_ENTRY){
          if(status == IR_SENSORS_RIGHT){
            motors_rotate_ccw(150);
          }else if(status == IR_SENSORS_LEFT){
            motors_rotate_cw(150);
          }else if(status == IR_SENSORS_NONE){
            motors_brake();
            ir_disable();
            charger_set_internal_count(1);
            //charger_next_stage();
          }
        }
        if(ir_streaming_enabled()){
          //NRF_LOG_RAW_INFO("%08d [main] IR DETEC TNONE\n", systemTimeGetMs());
          ble_indicate_status(DCS_IR_STATUS_UPDATED, &status, 1);
        }
      }
      break;

      case EVENT_GYRO_CALIBRATION_DONE:
      case EVENT_ACCEL_CALIBRATION_DONE:
        if(gyro_ready() && accel_ready()){
          uint8_t byte = 0xFF;
          ble_indicate_status(DCS_STATUS_CALIBRATION_DONE, &byte, 1);
          nav_update_pitch();
        }
      break;

      case EVENT_COLLISION:
        NRF_LOG_RAW_INFO("%08d [main] COLLISION DETECTED\n", systemTimeGetMs());
        led_rgb(1,1,1);
        nav_motion_update(MOTION_BRAKE, NULL, 0);
        
        if(accel_get_collision_detection_state() == COLLISIONS_DETECT_MOVING){
          nav_recover();
          nav_disable_collision_detection();
        }

        accel_disable_collision_detection();

        // Notify master
        uint8_t byte = 0xFF;
        ble_indicate_status(DCS_STATUS_COLLISION_DETECTED, &byte, 1);        
      break;

      default:
        NRF_LOG_RAW_INFO("%08d [main] unhandled event:%d\n", systemTimeGetMs(), eventQueueFront());
        break;
    }

    //if(eventQueueFront() != 33 && eventQueueFront()!= 27){
    //  NRF_LOG_RAW_INFO("%08d [main] DONE handling event %d\n", systemTimeGetUs(), eventQueueFront());
    //}
    eventQueuePop();
  }
}

/**
 * @}
 */
