#ifndef _I2S_H_
#define _I2S_H_

//#include "src/constants.h"
#include <stdint.h>
#include <nrf_gpio.h>

#define I2S_SCK_PIN   NRF_GPIO_PIN_MAP(0, 13) // BCLK (P0_13)
#define I2S_LCK_PIN   NRF_GPIO_PIN_MAP(1, 0) // WCLK (P1_00)
#define I2S_SDOUT_PIN  NRF_GPIO_PIN_MAP(0, 17)

#define SAMPLING_RATE (100000)
//#define CHUNK_SIZE (2 * (SAMPLING_RATE / 100))
//#define CHUNK_SIZE (2 * (SAMPLING_RATE / 100))

#define CHUNK_SIZE (3792)

typedef struct {
  int16_t *data; // Could be in ram or in flash
  uint16_t num_chunks;
  uint16_t current_chunk;
  uint16_t num_repetitions;
  uint16_t current_repetition;
  uint16_t length;
  bool finished;
} i2s_stream_t;

bool i2s_init(void);
bool i2s_deinit(void);

// Replaces current i2s stream object with another one with the given parameters.
// Stream cannot be replaced while it is ongoing.
bool i2s_load_stream(const int16_t * const buffer, uint16_t length, uint16_t repititions);

void i2s_reset(void);

// Starts playing a stream
void i2s_start(void);

void i2s_stop(void);

#endif