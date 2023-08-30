#ifndef __CHARGER_H__
#define __CHARGER_H__

#include <stdbool.h>
#include <stdint.h>

typedef enum{
  CHARGER_ENTRY_BEGIN = 0,
  CHARGER_ENTRY_ROTATION0,
  CHARGER_ENTRY_ROTATION1,
  CHARGER_ENTRY_ROTATION2,
  CHARGER_ENTRY_ROTATION3,
  CHARGER_ENTRY_FORWARD1,
  CHARGER_ENTRY_FORWARD2,
  CHARGER_ENTRY_DONE
} station_entry_state_name_t;

typedef  enum{
  CHARGER_UPDATE_STATE_HOLD,
  CHARGER_UPDATE_STATE_NEXT,
}charger_state_update_t; 

void charger_init(void);
bool is_charging(void);
void charger_update_internal_count(void);
void charger_start_station_entry(void);
void charger_next_stage(void);
void charger_set_internal_count(uint8_t val);


#endif