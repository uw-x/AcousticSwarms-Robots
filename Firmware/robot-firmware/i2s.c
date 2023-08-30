#include "i2s.h"
#include <nrfx_i2s.h>
#include <math.h>
#include <stdlib.h>

#include "nrf_log.h"
#include "timers.h"

static i2s_stream_t m_stream;
static uint32_t m_i2s_buffer[CHUNK_SIZE];
static int fix = 2;

void i2s_load_next_chunk(void){
  uint16_t length = CHUNK_SIZE;
  if(m_stream.current_chunk == m_stream.num_chunks - 1){
    length = m_stream.length % CHUNK_SIZE;
    if(length == 0) length = CHUNK_SIZE;
    //else{
    //  memset(m_i2s_buffer + length, 0, CHUNK_SIZE - length); 
    //}
  }

  //NRF_LOG_RAW_INFO("[%08lld] Chunk %d/%d, Length %d, Start %d\n", systemTimeGetUs(),
  //                                                     m_stream.current_chunk
  //                                                     , m_stream.num_chunks
  //                                                     , length
  //                                                     , CHUNK_SIZE * m_stream.current_chunk);
  
  //memcpy_fast(m_i2s_buffer, m_stream.data + CHUNK_SIZE * m_stream.current_chunk, sizeof(int16_t) * length);
  for(int i = 0; i < CHUNK_SIZE; i++){
    if (i < length) m_i2s_buffer[i] = *(m_stream.data + CHUNK_SIZE * m_stream.current_chunk + i);
    else m_i2s_buffer[i] = 0;
  }
  nrf_i2s_transfer_set(NRF_I2S, CHUNK_SIZE, NULL, (uint32_t * ) m_i2s_buffer);
  
  if(m_stream.current_chunk == 0 && fix < 2) {
    // Fixes weird issue where first two buffers in the first playback
    // are dropped for some reason
    fix++;
  }else{
    m_stream.current_chunk++;
  }
  
  if(m_stream.current_chunk == m_stream.num_chunks){
    m_stream.current_repetition++;
    m_stream.current_chunk = 0;
  }
}

void i2s_reset(void){
  m_stream.current_chunk = 0;
  m_stream.current_repetition = 0;
  m_stream.finished = true;
  nrf_i2s_enable(NRF_I2S);
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_RXPTRUPD);
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD);
  nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_STOPPED);
  nrf_i2s_int_enable(NRF_I2S,  NRF_I2S_INT_TXPTRUPD_MASK);
}

void nrfx_i2s_irq_handler(void){
  if(m_stream.finished){
    m_stream.finished = false;
  }
  
  if (nrf_i2s_event_check(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD)){
    nrf_i2s_event_clear(NRF_I2S, NRF_I2S_EVENT_TXPTRUPD);

    if(m_stream.current_repetition == m_stream.num_repetitions){
      // End current stream.
      nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_STOP);  
      // Workaround for Errata 194
      *((volatile uint32_t *)0x40025038) = 1;
      *((volatile uint32_t *)0x4002503C) = 1;

      nrf_i2s_int_disable(NRF_I2S, NRF_I2S_INT_TXPTRUPD_MASK);
      nrf_i2s_disable(NRF_I2S);

      m_stream.finished = true;
    }else{
      i2s_load_next_chunk();
    }
  }
}

bool i2s_load_stream(const int16_t * const buffer, uint16_t length, uint16_t repititions){
  // Check if stream is ongoing. If it is, then we don't replace it.
  if(!m_stream.finished) return false;

  m_stream.data = buffer;
  m_stream.length = length;
  m_stream.num_chunks = (length - 1) / CHUNK_SIZE + 1;
  m_stream.num_repetitions = repititions;

  i2s_reset();
}

void i2s_start(void){
  nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_START);
}

void i2s_stop(void){
  nrf_i2s_task_trigger(NRF_I2S, NRF_I2S_TASK_STOP);
}

static void configure_pins(nrfx_i2s_config_t const * p_config)
{
    uint32_t mck_pin, sdout_pin, sdin_pin;

    // Configure pins used by the peripheral:

    // - SCK and LRCK (required) - depending on the mode of operation these
    //   pins are configured as outputs (in Master mode) or inputs (in Slave
    //   mode).
    if (p_config->mode == NRF_I2S_MODE_MASTER)
    {
        nrf_gpio_cfg_output(p_config->sck_pin);
        nrf_gpio_cfg_output(p_config->lrck_pin);
    }
    else
    {
        nrf_gpio_cfg_input(p_config->sck_pin,  NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_input(p_config->lrck_pin, NRF_GPIO_PIN_NOPULL);
    }

    // - MCK (optional) - always output,
    if (p_config->mck_pin != NRFX_I2S_PIN_NOT_USED)
    {
        mck_pin = p_config->mck_pin;
        nrf_gpio_cfg_output(mck_pin);
    }
    else
    {
        mck_pin = NRF_I2S_PIN_NOT_CONNECTED;
    }

    // - SDOUT (optional) - always output,
    if (p_config->sdout_pin != NRFX_I2S_PIN_NOT_USED)
    {
        sdout_pin = p_config->sdout_pin;
        nrf_gpio_cfg_output(sdout_pin);
    }
    else
    {
        sdout_pin = NRF_I2S_PIN_NOT_CONNECTED;
    }

    // - SDIN (optional) - always input.
    if (p_config->sdin_pin != NRFX_I2S_PIN_NOT_USED)
    {
        sdin_pin = p_config->sdin_pin;
        nrf_gpio_cfg_input(sdin_pin, NRF_GPIO_PIN_NOPULL);
    }
    else
    {
        sdin_pin = NRF_I2S_PIN_NOT_CONNECTED;
    }

    nrf_i2s_pins_set(NRF_I2S,
                     p_config->sck_pin,
                     p_config->lrck_pin,
                     mck_pin,
                     sdout_pin,
                     sdin_pin);
}

bool i2s_init(void){
  ret_code_t err_code;
  nrfx_i2s_config_t cfg;
  cfg.sck_pin = I2S_SCK_PIN;
  cfg.lrck_pin = I2S_LCK_PIN;
  cfg.mck_pin = NRFX_I2S_PIN_NOT_USED;
  cfg.sdout_pin = I2S_SDOUT_PIN;
  cfg.sdin_pin =  NRFX_I2S_PIN_NOT_USED;
  cfg.irq_priority =  APP_IRQ_PRIORITY_LOW;
  cfg.mode = NRF_I2S_MODE_MASTER;
  cfg.format = NRF_I2S_FORMAT_I2S;
  cfg.alignment = NRF_I2S_ALIGN_LEFT;
  cfg.sample_width = NRF_I2S_SWIDTH_16BIT;
  cfg.channels = NRF_I2S_CHANNELS_STEREO;
  //cfg.channels = NRF_I2S_CHANNELS_LEFT;
  
  
  //// Sampling frequency = 100kHz
  cfg.mck_setup = NRF_I2S_MCK_32MDIV10;
  cfg.ratio = NRF_I2S_RATIO_32X;
  //// Sampling frequency = 62.5kHz
  //cfg.mck_setup = NRF_I2S_MCK_32MDIV8;
  //cfg.ratio = NRF_I2S_RATIO_64X;
  //// Sampling frequency = 50kHz
  //cfg.mck_setup = NRF_I2S_MCK_32MDIV10;
  //cfg.ratio = NRF_I2S_RATIO_64X;
  // Sampling frequency =~ 48kHz
  //cfg.mck_setup = NRF_I2S_MCK_32MDIV21;
  //cfg.ratio = NRF_I2S_RATIO_32X;

  if (!nrf_i2s_configure(NRF_I2S, cfg.mode, cfg.format, cfg.alignment,
                         cfg.sample_width, cfg.channels, cfg.mck_setup, cfg.ratio)){
      //err_code = NRFX_ERROR_INVALID_PARAM;
      //NRFX_LOG_WARNING("Function: %s, error code: %s.", __func__,
      //                 NRFX_LOG_ERROR_STRING_GET(err_code));
      return false;
  }
  configure_pins(&cfg);

  NRFX_IRQ_PRIORITY_SET(I2S_IRQn, cfg.irq_priority);
  NRFX_IRQ_ENABLE(I2S_IRQn);

  m_stream.finished = true;

  // Fix for possible nrf bug (TXPTRUPD asserted immediately first two times)
  //nrf_i2s_transfer_set(NRF_I2S, 1, NULL, (uint32_t*) m_i2s_buffer);
  //nrf_i2s_transfer_set(NRF_I2S, 1, NULL, (uint32_t*) m_i2s_buffer);

  return true;
  //return nrfx_i2s_init(&cfg, (nrfx_i2s_data_handler_t) i2s_data_handler) == NRF_SUCCESS;
}

bool i2s_deinit(void){
  *((volatile uint32_t *)0x40025038) = 1;
  *((volatile uint32_t *)0x4002503C) = 1;
  nrf_i2s_disable(NRF_I2S);
  NRFX_IRQ_DISABLE(I2S_IRQn);
  return true;
}