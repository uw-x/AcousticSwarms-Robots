#include "ts_tasks.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "nrf_egu.h"
#include "nrf_i2s.h"
#include "nrf_log.h"

#include "audio.h"
#include "i2s.h"
#include "event.h"
#include "timers.h"
#include "time_sync.h"
#include "ts_sync.h"
#include "led.h"
#include "buzzer.h"

#include "ble_manager.h"

#define TS_INTERVAL_DURATION_MS 50

#define TS_TASKS_DEBUG 0

static ts_task_handler_t task_handlers[MAX_NUM_TASKS];
static bool m_ts_enabled = false;

// Tasks must remain sorted in decreasing order
static ts_task_t tasks[MAX_TS_TASKS];
static ts_rtask_t rtasks[MAX_TS_TASKS];
static uint16_t m_tasks = 0;
static uint16_t m_rtasks = 0;
static bool m_running;
static uint32_t m_previous_interval = 0;

void ts_disable_on_subsequent_interval(nrf_ppi_channel_t channel){
  nrf_ppi_channel_include_in_group(channel, NRF_PPI_CHANNEL_GROUP1);
  //rb_put(&m_clear_on_next_interval, &channel, 1);
  //NRF_TIMER3->EVENTS_COMPARE[4] = 0;
}

#define METADATA_LENGTH 8
static uint8_t metadata[METADATA_LENGTH] = {0};
void capture_metadata(void){
  NRF_EGU0->TASKS_TRIGGER[0] = 1; // This should capture both timer 2 & 3 at the same time.
    
  uint32_t timer_lower = NRF_TIMER3->CC[3];
  uint32_t timer_upper = num_master_overflows();
  
  uint64_t timestamp = (((uint64_t) timer_upper) << 32) + timer_lower;

  metadata[0] = (timer_upper >> 24) & 0xFF;
  metadata[1] = (timer_upper >> 16) & 0xFF;
  metadata[2] = (timer_upper >> 8) & 0xFF;
  metadata[3] = (timer_upper) & 0xFF;

  metadata[4] = (timer_lower >> 24) & 0xFF;
  metadata[5] = (timer_lower >> 16) & 0xFF;
  metadata[6] = (timer_lower >> 8) & 0xFF;
  metadata[7] = timer_lower & 0xFF;
}

void ts_sync_task_send_metadata(void){
  //transmitDone = true; // reset
  ble_reset_tx_state();
  ble_manager_send_metadata(metadata, METADATA_LENGTH);
  //bleSendData(metadata, METADATA_LENGTH);
}

// ________________________________ MIC START TASK __________________________________ //
//void SWI0_EGU0_IRQHandler(void){
//    if (NRF_EGU0->EVENTS_TRIGGERED[3] != 0){
//        NRF_EGU0->EVENTS_TRIGGERED[3] = 0;

//        audioSetStreamStarted(true);
//    }
//}

void mic_start_task_init(void){
  // Setup endpoints so that, when enabled, mic starts at next timesync overflow
  nrf_ppi_channel_endpoint_setup(
          MIC_START_PPI_CHANNEL,
          TS_TIME_INTERVAL_BEGIN_EVENT,
          audioGetPdmStartTaskAddress());

  //nrf_ppi_fork_endpoint_setup(MIC_START_PPI_CHANNEL,
  //        nrf_swi_get);

  ts_disable_on_subsequent_interval(MIC_START_PPI_CHANNEL);
}

void mic_start_handler(uint32_t interval){  
  // Capture time at which channel is enabled.
  capture_metadata();
  
  // This should start pdm at next timer3 overflow.
  nrf_ppi_channel_enable(MIC_START_PPI_CHANNEL);
  //ts_disable_on_subsequent_interval(MIC_START_PPI_CHANNEL);
  //NRF_TIMER3->EVENTS_COMPARE[4] = 0;
  TS_ENABLE_INTERRUPT_ON_NEXT_INTERVAL;
  
  // Offload this to main so it doesn't take too much time
  eventQueuePush(EVENT_BLE_SEND_METADATA);

  if(audioStreamStarted()){
    led_rgb(1, 1, 0);
    NRF_LOG_RAW_INFO(" *** WARNING MIC WAS ALREADY STARTED *** \n");
  }  
}
// _________________________________________________________________________________ //

// _______________________________ MIC RECORD TASK _________________________________ //
void mic_record_task_init(void){
  // Nothing to do here
}

void mic_record_handler(uint32_t interval){
  audio_set_next_recording_beginning((interval - MASTER_OVERFLOW_TRIGGER_COUNT) * TS_INTERVAL_DURATION_MS);
  eventQueuePush(EVENT_RECORDER_RESET);
}

// _________________________________________________________________________________ //

// ________________________________ SPK START TASK _________________________________ //

void spk_start_task_init(){
  // Start at next timer3 overflow.
  nrf_ppi_channel_endpoint_setup(
            SPK_START_PPI_CHANNEL,
            TS_TIME_INTERVAL_BEGIN_EVENT,
            nrf_i2s_task_address_get(NRF_I2S, NRF_I2S_TASK_START));
  
  ts_disable_on_subsequent_interval(SPK_START_PPI_CHANNEL);
}

void spk_start_handler(uint32_t interval){
  //i2s_reset();
  buzzer_load_chirp();
  
  // Make sure to load the audio buffer first!!
  nrf_ppi_channel_enable(SPK_START_PPI_CHANNEL);
  TS_ENABLE_INTERRUPT_ON_NEXT_INTERVAL;
  //i2s_start();
}
// _________________________________________________________________________________ //


// Disable all PPIs on channel group 1.
void ts_sync_tasks_disable_pending_ppis(void){
  NRF_EGU0->TASKS_TRIGGER[2] = 1;
  //while(!rb_empty(&m_clear_on_next_interval))
  //  nrf_ppi_channel_disable(rb_get(&m_clear_on_next_interval));
}

// Be careful to arm tasks so that they start at the beginning
// of the NEXT interval
void ts_sync_tasks_init(void){
  // Link the EGU Master Channel trigger to enable Timer2 clear  
  nrf_ppi_channel_endpoint_setup(
      TS_MASTER_PPI_CHANNEL,
      (uint32_t) nrf_egu_event_address_get(NRF_EGU0, NRF_EGU_EVENT_TRIGGERED1),
      (uint32_t) nrf_timer_task_address_get(NRF_TIMER2, NRF_TIMER_TASK_CLEAR));

  nrf_ppi_channel_enable(TS_MASTER_PPI_CHANNEL);

  nrf_ppi_channel_endpoint_setup(
      TS_PENDING_DISABLE_PPI_CHANNEL,
      (uint32_t) nrf_egu_event_address_get(NRF_EGU0, NRF_EGU_EVENT_TRIGGERED2),
      (uint32_t) nrf_ppi_task_address_get(NRF_PPI_TASK_CHG1_DIS));

  nrf_ppi_channel_enable(TS_PENDING_DISABLE_PPI_CHANNEL);
  
  // Mic start task: Begins recording data on the microphone
  // and either streams or records it, based on configuration
  mic_start_task_init();
  task_handlers[TS_TASK_MIC_START] = mic_start_handler;

  // Mic record task: Saves data to memory (Exactly 50ms)
  mic_record_task_init();
  task_handlers[TS_TASK_MIC_RECORD] = mic_record_handler;

  // Speaker chirp task: Starts a chirp
  spk_start_task_init();
  task_handlers[TS_TASK_SPK_START] = spk_start_handler;

  m_running = false;

  //rb_create(&m_clear_on_next_interval, 32);

  // Enable interrupts on Timer3 to disable pending ppis.
  NVIC_EnableIRQ(TIMER3_IRQn);
}

void ts_sync_task_add(uint8_t id, uint32_t interval){
  interval = interval + MASTER_OVERFLOW_TRIGGER_COUNT;
  
  tasks[m_tasks].interval = interval;
  tasks[m_tasks].id = id;
  uint16_t i = m_tasks;
  
  // Bubble down
  while(i > 0 && tasks[i-1].interval < tasks[i].interval){
    ts_task_t temp = tasks[i-1];
    tasks[i-1] = tasks[i];
    tasks[i] = temp;
    i--;
  }

  m_tasks++;
#if TS_TASKS_DEBUG
  NRF_LOG_RAW_INFO("%08d [tsk_tasks] Added time-sync'd task %d at interval %d\n", systemTimeGetUs(), id, interval);
#endif
  //for(i = 0; i < m_tasks; i++){
  //  NRF_LOG_RAW_INFO("Task %d interval %d\n", systemTimeGetUs(), tasks[i].interval, tasks[i].id);
  //}
}

void ts_sync_task_add_recurring(uint8_t id, uint32_t period, uint32_t begin_interval, uint32_t end_interval){
  begin_interval = begin_interval + MASTER_OVERFLOW_TRIGGER_COUNT;
  end_interval = end_interval + MASTER_OVERFLOW_TRIGGER_COUNT;
  
  rtasks[m_rtasks].begin = begin_interval;
  rtasks[m_rtasks].end = end_interval;
  rtasks[m_rtasks].period = period;
  rtasks[m_rtasks].id = id;
  rtasks[m_rtasks].previous_triggered = -1;
  m_rtasks++;

#if TS_TASKS_DEBUG
  NRF_LOG_RAW_INFO("%08d [tsk_tasks] Added recurring time-sync'd task %d with period %d (%d, %d)\n", systemTimeGetUs(), id, begin_interval, end_interval);
#endif
  //for(i = 0; i < m_tasks; i++){
  //  NRF_LOG_RAW_INFO("Task %d interval %d\n", systemTimeGetUs(), tasks[i].interval, tasks[i].id);
  //}
}

#define absll(x) (x < 0 ? -x : x)
bool ts_tasks_process(uint32_t interval){
  //NRF_LOG_RAW_INFO("%08d [tsk_tasks] Interval: %d\n", systemTimeGetUs(), interval);
  
  
  if(interval < m_previous_interval){
#if TS_TASKS_DEBUG
    NRF_LOG_RAW_INFO("%08d [tsk_tasks] Interval: %d Previous Interval %d\n", systemTimeGetUs(), interval, m_previous_interval);
#endif
    led_rgb(1,1,0);
    //NRF_LOG_RAW_INFO(" *** WARNING MIC IS AHEAD (IGNORE IF NEAR END OF SYNC *** \n");
  }

  
  // Only process once per interval. Otherwise we're just wasting CPU resources.
  if(m_previous_interval >= interval) return 0 ;
  m_previous_interval = interval;
  
  ts_sync_update_ticks_ahead();
  
  // Handle one-time tasks
  while(m_tasks > 0 && tasks[m_tasks-1].interval /* + ts_master() */ == interval){
    //NRF_EGU0->TASKS_TRIGGER[0] = 1; // This should capture both timer 2 & 3 at the same time.
    
    //uint64_t timer_upper = NRF_TIMER2->CC[3];
    //uint64_t timer_lower = NRF_TIMER3->CC[3];
    //uint64_t timestamp = (((uint64_t) timer_upper)) * 800000 + timer_lower;
#if TS_TASKS_DEBUG
    NRF_LOG_RAW_INFO("%08d [tsk_tasks] Activated: %d, Interval: %d\n", systemTimeGetUs(), tasks[m_tasks - 1].id, tasks[m_tasks-1].interval);
#endif
    task_handlers[tasks[m_tasks - 1].id](interval); // trigger
    m_tasks--;
  }
  
  // Handle recurring tasks
  for(uint8_t i = 0; i < m_rtasks; i++){
    //NRF_LOG_RAW_INFO("%08d [tsk_tasks] Interval: %d, Checking task: %d, begin %d, end %d, period %d \n", systemTimeGetUs(), interval, rtasks[i].id, rtasks[i].begin, rtasks[i].end, rtasks[i].period);
    //NRF_LOG_RAW_INFO("%08d [tsk_tasks] Check1: %d ", systemTimeGetUs(), interval >= rtasks[i].begin)
    //NRF_LOG_RAW_INFO("\tCheck2: %d ", systemTimeGetUs(), interval < rtasks[i].end)
    //NRF_LOG_RAW_INFO("\tCheck3: %d ", systemTimeGetUs(), (interval - rtasks[i].begin) % rtasks[i].period == 0)
    //NRF_LOG_RAW_INFO("\tCheck4: %d ", systemTimeGetUs(), (rtasks[i].previous_triggered != interval))
    if(interval >= rtasks[i].begin
       && interval < rtasks[i].end 
       && (interval - rtasks[i].begin) % rtasks[i].period == 0
       && (rtasks[i].previous_triggered != interval)){
#if TS_TASKS_DEBUG
    NRF_LOG_RAW_INFO("%08d [tsk_tasks] Activated: %d, begin %d, end %d, period %d \n", systemTimeGetUs(), rtasks[i].id, rtasks[i].begin, rtasks[i].end, rtasks[i].period);
#endif
      task_handlers[rtasks[i].id](interval);
      rtasks[i].previous_triggered = interval;
    }
  }
  return 1;
}

void ts_tasks_set_role(ts_role role){
  switch(role){
    case TS_ROLE_OFF:
      m_ts_enabled = false;
      ts_sync_tasks_stop();
      ble_reset_sequence_number();
      ts_reset();
      led_rgb(0, 0, 1);
      NRF_LOG_RAW_INFO("%08d [main] *** NO TIMESYNC *** \n", systemTimeGetUs());
    break;
    case TS_ROLE_SLAVE:
      m_ts_enabled = true;
      NRF_LOG_RAW_INFO("%08d [main] time sync slave enabled\n", systemTimeGetUs());
      ts_sync_tasks_stop();
      led_rgb(1, 1, 1);   
    break;
    case TS_ROLE_MASTER:
      m_ts_enabled = true;
      NRF_LOG_RAW_INFO("%08d [main] time sync master enabled\n", systemTimeGetMs());
      led_rgb(0, 1, 0);
      // As soon as master is chosen, start time sync'd tasks on next interval
      TS_ENABLE_INTERRUPT_ON_NEXT_INTERVAL;
    break;
  }
}

bool ts_enabled(void){
  return m_ts_enabled;
}

void ts_sync_tasks_stop(void){
  ts_tx_stop();
  m_running = 0;
  m_rtasks = 0;
  m_tasks = 0;
}

bool ts_sync_tasks_started(void){
  return m_running;
}

void ts_sync_tasks_start(bool master){
  ts_sync_reset();
  
  NRF_EGU0->TASKS_TRIGGER[1] = 1; // Clear timer2 (set ts counter to 0)
  if(master){
    ts_tx_start(200); // Send sync packet every 1/200 s = 5ms
  }
  m_running = 1;
  
  m_previous_interval = 0;
}