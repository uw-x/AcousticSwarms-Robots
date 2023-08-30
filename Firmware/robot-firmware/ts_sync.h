#ifndef __TS_SYNC_H__
#define __TS_SYNC_H__

#include <stdint.h>

void ts_sync_reset(void);
int64_t ts_sync_get_ticks_ahead(void);
void ts_sync_update_ticks_ahead(void);

#endif