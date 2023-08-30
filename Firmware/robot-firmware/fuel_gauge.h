#ifndef _FUEL_GAUGE_H_
#define _FUEL_GAUGE_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensors.h"
#include "nrf_gpio.h"

#define MAX17048_SIZEREG                      ( 0x01u )
#define MAX17048_REV		    "1.0.0"
#define MAX17048_CAPTION     "MAX17048 " MAX17048_REV

#define MAX17048_SLEEP_ON                     ( 0x01u )  
#define MAX17048_SLEEP_OFF                    ( 0x00u )

#define MAX17048_VCELL                        ( 0x02u )  /*!< address register voltage*/
#define MAX17048_SOC                          ( 0x04u )
#define MAX17048_MODE                         ( 0x06u )
#define MAX17048_VERSION                      ( 0x08u )
#define MAX17048_CONFIG                       ( 0x0Cu )
#define MAX17048_ID                           ( 0x18u )
#define MAX17048_CRATE                        ( 0x16u )
#define MAX17048_STATUS                        ( 0x1Au )

#define MAX17048_CMD                          ( 0xFEu  )
#define MAX17048_RESET                        ( 0x5400 )

#define MAX17048_ADDR_SLAVE                   ( 0x36u )
#define MAX17048_RCOMP0                       ( 0x97u )

/*MODE Register Format*/
#define MAX17048_MODE_HI_STAT_BIT             ( 12 )
#define MAX17048_MODE_EN_SLEEP_BIT	        ( 13 )
#define MAX17048_MODE_QUICK_START_BIT	        ( 14 )

// STATUS
#define MAX17048_RI_BIT (0 + 8)
#define MAX17048_VH_INT_BIT (1 + 8)
#define MAX17048_VL_INT_BIT (2 + 8)
#define MAX17048_VR_INT_BIT (3 + 8)
#define MAX17048_HD_INT_BIT (4 + 8)
#define MAX17048_SC_INT_BIT (5 + 8)
#define MAX17048_EnVr_INT_BIT (6 + 8)

/* CONFIG Register Format*/
#define MAX17048_CONFIG_ALRT_BIT                        ( 5 )
#define MAX17048_CONFIG_ALSC_BIT                        ( 6 )
#define MAX17048_CONFIG_SLEEP_BIT	        	( 7 )

/*Resolution for calculate Vcell*/
#define MAX17048_VCELL_RESOLUTION	            ( 78125 )


#define FG_INT_PIN NRF_GPIO_PIN_MAP(0, 28)


bool fg_init(void);

bool fg_test(void);
float fg_get_current_battery_level(void);
float fg_get_current_soc(void);
float fg_get_current_crate(void);
float fg_get_motor_compensation(void);

sensor_t* fg_get_sensor_class(void);
void fg_update_current_measurements(void);

void fg_handle_interrupt(void);
bool fg_stream_enabled(void);


#endif