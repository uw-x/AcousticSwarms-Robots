#ifndef __PROFILE_H__
#define __PROFILE_H__

#include <stdint.h>

#define PROFILE_TOF 0
#define NUM_PROFILES 1


typedef void (*profile_t) (const uint8_t * const params, uint16_t length);

void profile_init(void);
void profile_load(uint8_t id, const uint8_t *params, uint16_t length);


#endif
