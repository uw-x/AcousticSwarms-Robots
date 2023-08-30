#ifndef _TS_TASKS_H_
#define _TS_TASKS_H_

#include <stdint.h>
#include "nrf_ppi.h"


// Time sync
#define MASTER_OVERFLOW_TRIGGER_COUNT 3
#define MIC_START_PPI_CHANNEL NRF_PPI_CHANNEL5
#define TS_MASTER_PPI_CHANNEL NRF_PPI_CHANNEL7
#define SPK_START_PPI_CHANNEL NRF_PPI_CHANNEL8
#define TS_PENDING_DISABLE_PPI_CHANNEL NRF_PPI_CHANNEL9
#define SPK_SYNC_DEBUG NRF_PPI_CHANNEL10
#define TS_ENABLE_INTERRUPT_ON_NEXT_INTERVAL NRF_TIMER3->EVENTS_COMPARE[4] = 0;\
                                             NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE4_Set << TIMER_INTENSET_COMPARE4_Pos;

#define TS_TIME_INTERVAL_BEGIN_EVENT ((uint32_t) nrf_timer_event_address_get(NRF_TIMER3, NRF_TIMER_EVENT_COMPARE4))

#define MAX_TS_TASKS 64
#define MAX_NUM_TASKS 32

typedef void (*ts_task_handler_t) (uint32_t interval);

typedef enum{
  TS_TASK_MIC_START = 0,
  TS_TASK_MIC_RECORD,
  TS_TASK_SPK_START,
} ts_task_id_t;


typedef struct{
  uint8_t id;
  uint32_t interval;
} ts_task_t;

typedef struct{
  uint8_t id;
  uint32_t begin, end, period, previous_triggered;
} ts_rtask_t;

typedef enum{
  TS_ROLE_OFF,
  TS_ROLE_SLAVE,
  TS_ROLE_MASTER,
} ts_role;

void ts_sync_tasks_start(bool master);
void ts_sync_tasks_stop(void);
void ts_sync_tasks_init(void);

// Start a task at the beginning of interval + 1 timesync intervals
void ts_sync_task_add(uint8_t id, uint32_t interval);
bool ts_tasks_process(uint32_t interval);
void ts_sync_task_add_recurring(uint8_t id, uint32_t period, uint32_t begin_interval, uint32_t end_interval);

bool ts_sync_tasks_started(void);
void ts_sync_tasks_disable_pending_ppis(void);

void ts_tasks_set_role(ts_role role);
bool ts_enabled(void);

void ts_sync_task_send_metadata(void);
void ts_reset(void);

#endif