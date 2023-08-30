#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include "event.h"
#include "nrf_log.h"

#define EVENT_QUEUE_DEPTH 256

static uint8_t head = 0;
static uint8_t tail = 0;
static event_t eventQueue[EVENT_QUEUE_DEPTH] = {0};

void eventQueueInit(void) { head = tail = 0; }
bool eventQueueEmpty(void) { return (head == tail); }
event_t eventQueueFront(void) { return eventQueue[head]; }

event_t eventQueuePop(void)
{
  event_t event = eventQueue[head];
  head = (head + 1) % EVENT_QUEUE_DEPTH;
  return event;
}

void eventQueuePush(event_t event)
{
  //NRF_LOG_RAW_INFO("Event %d push into queue\n", event);
  eventQueue[eventQueueEmpty() ? head : tail]  = event;
  tail = (tail + 1) % EVENT_QUEUE_DEPTH;
  assert(head != tail); // overflow
}