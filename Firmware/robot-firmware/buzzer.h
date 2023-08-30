#ifndef _BUZZER_H_
#define _BUZZER_H_

#include <stdint.h>
#include <nrf_delay.h>


bool buzzer_init(void);
void buzzer_play_chirp(uint32_t nrepitions);
void buzzer_load_chirp(void);

#endif