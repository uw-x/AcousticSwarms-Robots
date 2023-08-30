#ifndef _TIMERS_H_
#define _TIMERS_H_

#define SYSTEM_TIMER            NRF_TIMER1
#define SYSTEM_TIMER_IRQn       TIMER1_IRQn
#define SYSTEM_TIMER_IRQHandler TIMER1_IRQHandler

uint32_t systemTimeGetMs(void);
uint64_t systemTimeGetUs(void);
uint64_t systemTimeGetTicks(void);
void delayMs(uint32_t);
void timersInit(void);

#endif