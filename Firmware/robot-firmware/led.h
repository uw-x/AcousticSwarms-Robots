#ifndef _LED_H_
#define _LED_H_

#define LED_ENABLED 1


typedef  enum{
  LED_OFF = 0,
  LED_ADVERTISING,
  LED_CONNECTED,
  LED_MASTER,
} led_state_t;


void led_init();
void led_rgb(bool r, bool g, bool b);
void led_deinit(void);

ret_code_t led_indicate(led_state_t state);


#endif
