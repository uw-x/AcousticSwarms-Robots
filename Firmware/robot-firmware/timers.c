#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>

#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_drv_clock.h"

#include "event.h"
#include "timers.h"

static uint32_t systemTimeSeconds;

static void systemTimerInit(void)
{
  systemTimeSeconds = 0;
  SYSTEM_TIMER->BITMODE = TIMER_BITMODE_BITMODE_24Bit << TIMER_BITMODE_BITMODE_Pos; // Ensure the timer uses 24-bit bitmode or higher
  SYSTEM_TIMER->PRESCALER = 0;                                                      // Set the prescaler to 4, for a timer interval of 1 us (16M / 2^4)
  SYSTEM_TIMER->CC[0] = 16000000;                                                    // Set the CC[0] register to hit after 1 second
  SYSTEM_TIMER->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;                           // Make sure the timer clears after reaching CC[0]
  SYSTEM_TIMER->INTENSET = TIMER_INTENSET_COMPARE0_Msk;                             // Trigger the interrupt when reaching CC[0]
  NVIC_SetPriority(SYSTEM_TIMER_IRQn, 7);                                           // Set a low IRQ priority and enable interrupts for the timer module
  NVIC_EnableIRQ(SYSTEM_TIMER_IRQn);
  SYSTEM_TIMER->TASKS_CLEAR = 1;                                                    // Clear and start the timer
  SYSTEM_TIMER->TASKS_START = 1;
}

uint32_t systemTimeGetMs(void)
{
  SYSTEM_TIMER->TASKS_CAPTURE[1] = 1;
  return (systemTimeSeconds * 1000) + (SYSTEM_TIMER->CC[1] / 16000);
}

uint64_t systemTimeGetUs(void)
{
  SYSTEM_TIMER->TASKS_CAPTURE[1] = 1;
  return (uint64_t)systemTimeSeconds * 1000000 + (SYSTEM_TIMER->CC[1]/16);
}

uint64_t systemTimeGetTicks(void)
{
  SYSTEM_TIMER->TASKS_CAPTURE[1] = 1;
  return (uint64_t)systemTimeSeconds * 16000000 + SYSTEM_TIMER->CC[1];
}

void SYSTEM_TIMER_IRQHandler(void)
{
  //NRF_LOG_INFO("[timers] IRQHANDLER");
  if(SYSTEM_TIMER->EVENTS_COMPARE[0]) {
    SYSTEM_TIMER->EVENTS_COMPARE[0] = 0;
    systemTimeSeconds++;
    eventQueuePush(EVENT_TIMERS_ONE_SECOND_ELAPSED);
  }
}

void delayMs(uint32_t delay)
{
  uint64_t now = systemTimeGetUs();
  while (systemTimeGetUs() < (now +(delay * 1000))) {
    __WFE();
  }
}

void delayUs(uint32_t delay)
{
  uint64_t now = systemTimeGetUs();
  while (systemTimeGetUs() < (now +(delay))) {
    __WFE();
  }
}

void timersInit(void)
{
  // Initialize timer module.
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  systemTimerInit();

  // Create timers.
  // err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
  // APP_ERROR_CHECK(err_code); */
  // err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
  // APP_ERROR_CHECK(err_code); */
}