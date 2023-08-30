#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include "stdint.h"
#include "stdbool.h"

typedef struct{
  uint32_t size;
  uint8_t *buf;
  uint32_t head, tail;
  uint8_t val;
} ringbuffer_t;

// Allocates memory for a ringbuffer and initializes it
void rb_create(ringbuffer_t *rb, uint32_t size);

// Initializes a ringbugger from pre-allocated buffer
void rb_init(ringbuffer_t *rb, uint8_t *buf, uint32_t size);

bool rb_put(ringbuffer_t *rb, const uint8_t *const data, uint32_t length);
bool rb_empty(ringbuffer_t *rb);
void rb_reset(ringbuffer_t *rb);

bool rb_put_w(ringbuffer_t *rb, uint8_t *data, uint16_t length, uint8_t metadata);

uint32_t rb_size(ringbuffer_t *rb);
uint8_t rb_get(ringbuffer_t *rb);
bool rb_get_fast(ringbuffer_t *rb, uint8_t *buf, uint32_t length);
bool rb_get_fast_offset(ringbuffer_t *rb, uint8_t *buf, uint32_t length, uint32_t offset);

#define rb_at(rb, i) \
  (rb)->buf[((rb)->head + i)%(rb)->size]

#define rb_advance(rb, i) \
  (rb)->head = ((rb)->head + i) % (rb)->size




#endif