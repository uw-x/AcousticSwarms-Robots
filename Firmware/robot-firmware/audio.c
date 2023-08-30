#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "math.h"
#include "arm_math.h"

#include <nrfx.h>

#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
//#include "arm_const_structs.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"
#include "nrfx_pdm.h"

#include "time_sync.h"
#include "ts_sync.h"
#include "timers.h"
#include "draw.h"
#include "gpio.h"
#include "event.h"
#include "audio_config.h"
#include "audio.h"
#include "ble_config.h"
#include "led.h"
#include "ble_manager.h"
#include "ble_dcs.h"
#include "ringbuffer.h"

#include <opus.h>


#define max(a, b) (a > b ? a : b);
#define min(a, b) (a < b ? a : b);

#define OPUS_BITRATE 32000

#define RECORD_TASK_DEBUG 1
#define AUDIO_PARAMS_DEBUG 1
//#define AUDIO_SYNC_DEBUG3
#define TIME_SYNC_DISABLED 0

#define PDM_EDGE_RISING  1
#define PDM_EDGE_FALLING 0


static OpusEncoder *encoder;
static OpusDecoder *decoder;

// Number of ticks per 1 sample. To be recompute after every freq change.
static uint16_t m_ticks_per_sample;

// Max tick offset before timesync action is taken
// Used to be equal to ticks per sample, now equal to 0.6 * m_ticks_per_sample
static uint16_t TICKS_THRESHOLD;

static uint16_t m_decimation_factor = 1;
static uint8_t m_pdm_ratio = 0;
static int16_t releasedPdmBuffer[PDM_MAX_BUFFER_LENGTH + 3] = {0};
static uint8_t encoded_buffer[2000] = {0};
static int16_t check[1000] = {0};
static int channels;

const int32_t opus_freqs[5] = {8000, 12000, 16000, 24000, 48000};
static uint16_t current_opus_window_size;
static uint8_t best_freq; // OPUS freq id

int16_t pdmBuffer[2][PDM_MAX_BUFFER_LENGTH+2] = {0}; // add two to the buffer for sample compensationReady         = false;

static uint16_t buffer_length;

static int pdmBufferIndex               = 0;
static int64_t samplesCompensated       = 0;
static volatile bool streamStarted = false;
uint16_t fakeData = 0;
uint64_t pdmBufferCount = 0;
uint64_t pdmSampleCount = 0;
uint64_t pdmDroppedCount = 0;
static mic_mode_t current_mic_mode = AUDIO_MODE_NONE;
static bool m_ts_disabled = false;
static uint32_t m_compression_bitrate = OPUS_BITRATE;

static nrfx_pdm_config_t current_params = {0};
static bool m_use_compression = false;


//#define RECORDING_DURATION_MS 250
#define RECORDING_DURATION_MS 50
static int64_t audio_record_time_start=-1; // T = 1.9 seconds @ 50000Hz
static int64_t audio_record_sample_start=-99999; // T = 1.9 seconds @ 50000Hz
static int64_t audio_record_sample_end=-99999; // T = 400ms @ 50000Hz
static bool m_audio_recording_finished;

static double current_freq = 0;
static double sample_time = 0;

static uint8_t audio_queue[AUDIO_QUEUE_MAX_SIZE][PACKET_LENGTH + 1];
static uint8_t audio_queue_head = 0;
static uint8_t audio_queue_tail = 0;
static uint8_t audio_queue_idx = 0;

#define OPUS_RB_SIZE (4500)
static int16_t opus_rb_buf[OPUS_RB_SIZE];
static ringbuffer_t opus_rb;

void audio_reset(void){
  audio_queue_head = 0;
  audio_queue_tail = 0;
  audio_queue_idx = 0;

  pdmDroppedCount = 0;
  pdmBufferCount = 0;
  pdmSampleCount = 0;
  
  m_audio_recording_finished = true;
  m_ts_disabled = false;

  rb_reset(&opus_rb);
}

bool audio_queue_empty(){
  return (AUDIO_QUEUE_MAX_SIZE - audio_queue_head + audio_queue_tail)
           % AUDIO_QUEUE_MAX_SIZE == AUDIO_QUEUE_MAX_SIZE - 1;
}

static inline void audio_queue_next(){
  audio_queue_idx = 0;
  audio_queue_tail = (audio_queue_tail + 1) % AUDIO_QUEUE_MAX_SIZE;
  // Drop earliest packet
  if(audio_queue_head == audio_queue_tail) {
    NRF_LOG_RAW_INFO("WARNING! A previous buffer has been overwritten!");
    led_rgb(1,0,1);
    pdmDroppedCount += PACKET_LENGTH;
    audio_queue_head = (audio_queue_head + 1) % AUDIO_QUEUE_MAX_SIZE;
  }
}

static inline bool audio_queue_put_sample(int16_t sample){
  audio_queue[audio_queue_tail][audio_queue_idx++] = sample;
  if(audio_queue_idx == PACKET_LENGTH + 1){
    audio_queue_next();
    return true;
  }
  return false;
}

uint8_t audioGetMicData(uint8_t **data){
  if(audio_queue_head != audio_queue_tail){
    *data = audio_queue[audio_queue_head];
    uint8_t length = audio_queue[audio_queue_head][PACKET_LENGTH];
    audio_queue_head = (audio_queue_head + 1) % AUDIO_QUEUE_MAX_SIZE;
    return length;
  }
  return 0;
}

uint16_t audio_buffer_length(void){
  return buffer_length / m_decimation_factor;
}

static void decimate(int16_t* outputBuffer, int16_t* inputBuffer, uint8_t decimationFactor){
  uint16_t length = audio_buffer_length();
  for (int i = 0; i < length; i++) {
    //if (fakeData < 32768) {
    if (fakeData < 10) {
      outputBuffer[i] = fakeData ++;
    } else {
#ifdef AUDIO_SYNC_DEBUG3
      outputBuffer[i] = fakeData ++;
#else
    outputBuffer[i] = inputBuffer[i*decimationFactor];
#endif
    }  
  }
}

uint8_t audio_queue_fill(uint8_t *buff, uint16_t length){
  uint8_t *ptr = buff;
  uint16_t amount;
  uint8_t ready = 0;
  while(length > 0){
    amount = min(length, PACKET_LENGTH - ((int16_t) audio_queue_idx));
    memcpy_fast(audio_queue[audio_queue_tail] + audio_queue_idx, ptr, amount);
    
    audio_queue_idx += amount;
    
    //NRF_LOG_RAW_INFO("[audio] Copied %d bytes, idx = %d\n", amount, audio_queue_idx);
    if(audio_queue_idx == PACKET_LENGTH){
      ready += audio_queue_put_sample(PACKET_LENGTH);
      //NRF_LOG_RAW_INFO("[audio] READY %d\n", ready);
    }

    ptr += amount;
    length -= amount;
  }
  return ready;
}

static int n_packets = 0;

static void pdmEventHandler(nrfx_pdm_evt_t *event)
{
  nrfx_err_t errorStatus;
  static bool pdmBufferSwitchFlag = false;

  if (event->error != NRFX_PDM_NO_ERROR) {
    NRF_LOG_RAW_INFO("[audio] pdm error\n");
    ASSERT(0);
  } 

  if (event->buffer_released) {
    if (!audioStreamStarted()) {
      audioSetStreamStarted(true);
    }
    //NRF_LOG_RAW_INFO("%08d [audio] buffer length: %d, modchannels: %d\n", systemTimeGetUs(), audio_buffer_length(), audio_buffer_length() % channels);
    APP_ERROR_CHECK_BOOL((audio_buffer_length() % channels) == 0);
    int16_t length = audio_buffer_length() / channels;

    //gpioWrite(GPIO_3_PIN, (pdmBufferIndex == 0) ? 1 : 0);
    //decimate(releasedPdmBuffer, event->buffer_released, m_decimation_factor);
    int64_t t1 = systemTimeGetUs();
    memcpy_fast(releasedPdmBuffer, event->buffer_released, sizeof(uint16_t) * length * channels);

    // If current sampling rate is not the same as opus rate, resample
    if(m_use_compression && current_opus_window_size != buffer_length){
      audio_resample(releasedPdmBuffer, buffer_length, current_opus_window_size, channels);
      length = current_opus_window_size;
    }
    // if ticksAhead > TICKS_THRESHOLD, increase pdm buffer size to slow down
    // if ticksAhead < -TICKS_THRESHOLD, decrease pdm buffer size to catch up
    int8_t bufferTweakAmount = 0;
    if (!ts_master() && streamStarted) {
      if (ts_sync_get_ticks_ahead() > TICKS_THRESHOLD + samplesCompensated * m_ticks_per_sample) { 
          bufferTweakAmount = -1; // Remove one sample
      } else if (ts_sync_get_ticks_ahead() < - TICKS_THRESHOLD + samplesCompensated * m_ticks_per_sample) {
          bufferTweakAmount = 1; // Add one sample
      }
      
      if (bufferTweakAmount != 0) {
          NRF_LOG_RAW_INFO("%08d [audio] samplesCompensated:%d bufferTweakAmount:%d\n",
            systemTimeGetMs(), samplesCompensated, bufferTweakAmount);
          #if TIME_SYNC_DISABLED
                    bufferTweakAmount = 0;
                    eventQueuePush(EVENT_METADATA_SAVE_TIMESTAMP);
          #endif
      }
      
      if(m_ts_disabled){
        bufferTweakAmount = 0;
      }else{
        samplesCompensated -= bufferTweakAmount;
      }
    }
    length += bufferTweakAmount;

    pdmSampleCount += length;

    uint8_t ready = 0; // Amount of new buffers added to the queue

    //int64_t t1 = systemTimeGetUs();
    if(m_use_compression){
      APP_ERROR_CHECK_BOOL(channels == 1); // Compression is only supported in single channel mode.
      //NRF_LOG_RAW_INFO("Opus window size samples: %d\n", current_opus_window_size);
      
      if(!rb_put(&opus_rb, (uint8_t*) releasedPdmBuffer, sizeof(int16_t) * length * channels)){
        NRF_LOG_RAW_INFO("****WARNING: OPUS RB FULL!****\n");
      }

      while(rb_size(&opus_rb) >= 2 * current_opus_window_size * channels){

        for(int i = 0; i < current_opus_window_size * channels; i++){
          int16_t val;
          ((uint8_t*) &val)[0] = rb_get(&opus_rb);
          ((uint8_t*) &val)[1] = rb_get(&opus_rb);
          releasedPdmBuffer[i] = val;
        }

        int16_t nbytes = opus_encode(encoder, releasedPdmBuffer, current_opus_window_size, encoded_buffer, 2000);
        if(nbytes < 0){
          NRF_LOG_RAW_INFO("[audio] encode failed: %s\n", opus_strerror(nbytes));
        }else{
          n_packets++;

          //NRF_LOG_RAW_INFO("[audio] encoded bytesize: %d\n", nbytes);

          uint16_t length = nbytes;
          uint8_t length_upper = (length>>8) & 0xFF;
          uint8_t length_lower = length & 0xFF;
          ready = audio_queue_fill(&length_upper, 1);
          ready |= audio_queue_fill(&length_lower, 1);
          ready |= audio_queue_fill(encoded_buffer, length);

        }
      }
    }else{
      ready = audio_queue_fill((uint8_t *) releasedPdmBuffer, sizeof(int16_t) * length * channels);
      
    }

    int64_t t2 = systemTimeGetUs();

    //NRF_LOG_RAW_INFO("[audio] Time taken to put in queue: %lld\n", t2 - t1);

    if(ready){
      //NRF_LOG_RAW_INFO("[audio] Placed %d buffers in audio queue\n", ready);
      //NRF_LOG_RAW_INFO("[audio] Sample count: %d\n", pdmSampleCount);
      eventQueuePush(EVENT_AUDIO_MIC_DATA_READY);
    }
    //NRF_LOG_RAW_INFO("%08d [audio] DONE buffer released\n", systemTimeGetUs());
  }

  if (event->buffer_requested) {
    pdmBufferIndex = (pdmBufferIndex == 0) ? 1 : 0;
    errorStatus = nrfx_pdm_buffer_set(pdmBuffer[pdmBufferIndex], buffer_length);
    ASSERT(errorStatus == NRFX_SUCCESS);
  }

  return;
}

uint32_t audioGetPdmBufferCount(void)
{
  return pdmBufferCount;
}

uint64_t audioGetPdmSampleCount(void)
{
  return pdmSampleCount;
}

bool audioStreamStarted(void)
{
  return streamStarted;
}

void audioSetStreamStarted(bool started)
{
  if(started){
    NRF_LOG_RAW_INFO("%08d [audio] Started!\n", systemTimeGetUs())
  }
  ble_reset_tx_state();
  streamStarted = started;
}

uint32_t audioGetPdmStartTaskAddress(void)
{
  return nrfx_pdm_task_address_get(NRF_PDM_TASK_START);
}

//static uint32_t GetFreeMemorySize(){
//  uint32_t  i;
//  uint32_t  len;
//  uint8_t*  ptr;
 
//  for(i=1;;i++){
//    len = i * 1000;
//    ptr = (uint8_t*)malloc(len);
//    if (!ptr)
//      break;
//    free(ptr);
//  }
 
//  return len - 1000;
//}


void audioInit(void){
  nrf_gpio_cfg_output(MIC_EN_PIN);

  m_ts_disabled = false;

  rb_init(&opus_rb, (uint8_t*) opus_rb_buf, OPUS_RB_SIZE * sizeof(int16_t));
  
  // DIV32: 0x08000000 -> CLK: 1.000 MHz -> SR: 15625 Hz
  // DIV31: 0x08400000 -> CLK: 1.032 MHz -> SR: 16129 Hz
  // DIV30: 0x08800000 -> CLK: 1.067 MHz -> SR: 16667 Hz
  // DIV25: 0x0A000000 -> CLK: 1.280 MHz -> SR: 20000 Hz
  // DIV16: 0x10000000 -> CLK: 2.000 MHz -> SR: 31250 Hz
  // DIV12: 0x15000000 -> CLK: 2.667 MHz -> SR: 41667 Hz
  // DIV10: 0x19000000 -> CLK: 3.200 MHz -> SR: 50000 Hz
  // DIV08: 0x20000000 -> CLK: 4.000 MHz -> SR: 62500 Hz
  audio_params_t params;
  params.div_clk = 0x08400000;
  params.decimation_factor = 1;
  params.mode = NRF_PDM_MODE_MONO;
  params.left_gain = NRF_PDM_GAIN_MAXIMUM;
  params.right_gain = NRF_PDM_GAIN_MAXIMUM;
  params.use_compression = false; // By default, don't use compression
  params.ratio = 0; // RATIO = 64

  audio_params_update(&params);

  //NRF_LOG_RAW_INFO("%08d [audio] initialized\n", systemTimeGetMs());
}

void audio_ready(void){
  if(audioStreamStarted()){
    NRF_LOG_RAW_INFO("%08d [audio]  ****** WARNING 1 ****** \n", systemTimeGetMs());
    led_rgb(0, 1, 1);
  }
  
  NRF_LOG_RAW_INFO("%08d [audio] AUDIO READY\n", systemTimeGetMs());
  
  // Remove everything in audio queue
  audio_reset();

  nrfx_err_t errorStatus = NRF_SUCCESS;
  errorStatus = nrfx_pdm_init(&current_params, (nrfx_pdm_event_handler_t) pdmEventHandler);
  ASSERT(errorStatus == NRFX_SUCCESS);

  // Update channels
  channels = 2 - current_params.mode;

  // Update PDM ratio
  NRF_PDM->RATIO = m_pdm_ratio;

  current_freq = (32000000 / ((((uint64_t) 1048576) * 4096) / current_params.clock_freq) / (m_pdm_ratio?80:64) );

  buffer_length = ((int16_t) round(PDM_BUFFER_DURATION * current_freq / 1000)) * channels;

  // Compute closest opus frequency given effective sampling rate.
  best_freq = 0;
  for(uint8_t i = 1; i < 5; i++){
    //NRF_LOG_RAW_INFO("%08d [audio] try: %d, Distance: =%d\n Previous best = %d", systemTimeGetMs(), opus_freqs[i], abs(current_freq - opus_freqs[i]), abs(current_freq - opus_freqs[best_freq]));
    if(abs(current_freq - opus_freqs[best_freq]) > abs(current_freq - opus_freqs[i])){
      best_freq = i;
    }
  }
  current_opus_window_size = round(opus_freqs[best_freq] * PDM_BUFFER_DURATION / 1000.0);

  if(m_use_compression){
    sample_time = 1.0 / opus_freqs[best_freq];
  }else{
    sample_time = 1.0 / current_freq;
  }
  // Sample time * time sync clock frequency
  m_ticks_per_sample = (uint16_t) round(sample_time * 16000000);
  TICKS_THRESHOLD = 0.6 * m_ticks_per_sample;
  
#if AUDIO_PARAMS_DEBUG
  NRF_LOG_RAW_INFO("%08d [audio] opus freq=%d\n", systemTimeGetMs(), opus_freqs[best_freq]);

  NRF_LOG_RAW_INFO("%08d [audio] current_freq=" NRF_LOG_FLOAT_MARKER "\n", systemTimeGetMs(), NRF_LOG_FLOAT(current_freq));
  NRF_LOG_RAW_INFO("%08d [audio] sample_time=" NRF_LOG_FLOAT_MARKER "\n", systemTimeGetMs(), NRF_LOG_FLOAT(sample_time));
  NRF_LOG_RAW_INFO("%08d [audio] ticks threshold=%d\n", systemTimeGetMs(), TICKS_THRESHOLD);
  NRF_LOG_RAW_INFO("%08d [audio] use compression=%d\n", systemTimeGetMs(), m_use_compression);
  NRF_LOG_RAW_INFO("%08d [audio] compression bitrate=%d\n", systemTimeGetMs(), m_compression_bitrate);
  NRF_LOG_RAW_INFO("%08d [audio] num channels=%d\n", systemTimeGetMs(), channels);
#endif

  // Update encoder parameters
   //Worth trying different applications?
  int err = 0;

  // Initialize encoder
  if(m_use_compression){
    int size = opus_encoder_get_size(channels);

    if(encoder == NULL){
      encoder = malloc(size);
    }else{
      APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_RESET_STATE) == OPUS_OK);
    }

    //size = opus_decoder_get_size(channels);
    //decoder = malloc(size);

    if(encoder == NULL){
      NRF_LOG_RAW_INFO("%08d [audio] Failed to allocate memory (%d bytes) for encoder.\n", systemTimeGetMs(), size);
    }

    err = opus_encoder_init(encoder, opus_freqs[best_freq], 2 - current_params.mode, OPUS_APPLICATION_RESTRICTED_LOWDELAY);
    if (err<0){
      NRF_LOG_RAW_INFO("failed to initialize encoder: %s\n", opus_strerror(err));
      return;
    }
    err = opus_encoder_ctl(encoder, OPUS_SET_BITRATE(m_compression_bitrate ));

    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_LSB_DEPTH(16)) == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0)) == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_DTX(0)) == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(0)) == OPUS_OK);
    APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(0)) == OPUS_OK);
  
    //int gain = 0;
    //opus_decoder_ctl(decoder, OPUS_GET_GAIN(&gain));
    //NRF_LOG_RAW_INFO("GAIN: %d", gain);

    //APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_BITRATE(OPUS_AUTO))                      == OPUS_OK);
    //APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_VBR(1))                              == OPUS_OK);
    //APP_ERROR_CHECK_BOOL(opus_encoder_ctl(encoder, OPUS_SET_VBR_CONSTRAINT((0)))== OPUS_OK);

    if (err<0){
      NRF_LOG_RAW_INFO("failed to set bitrate: %s\n", opus_strerror(err));
      return;
    }
  }
  //err = opus_decoder_init(decoder, opus_freqs[best_freq], 2 - current_params.mode);
  //if (err<0){
  //  NRF_LOG_RAW_INFO("failed to initialize decoder: %s\n", opus_strerror(err));
  //  return;
  //}
  //NRF_LOG_RAW_INFO("Created Decoder!\n");

  nrfx_pdm_buffer_set(pdmBuffer[0], buffer_length);
  ASSERT(errorStatus == NRFX_SUCCESS);

#ifdef AUDIO_PARAMS_DEBUG
  NRF_LOG_RAW_INFO("%08d [audio] Created encoder\n", systemTimeGetMs());

  NRF_LOG_RAW_INFO("%08d [audio] Buffer Length: %d\n", systemTimeGetMs(), buffer_length);
#endif

  // Reset synchronization
  samplesCompensated = 0;

  nrf_gpio_pin_set(MIC_EN_PIN);
  delayMs(1);

  if(audioStreamStarted()){
    NRF_LOG_RAW_INFO("%08d [audio]  ****** WARNING 2 ****** \n", systemTimeGetMs());
    led_rgb(0, 1, 1);
  }
}

uint8_t audio_get_channels(void){
  return channels;
}

void audio_params_update(audio_params_t *params){
  // Setup PDM
  nrfx_pdm_config_t pdmConfig = current_params;
  pdmConfig.edge               = (nrf_pdm_edge_t)PDM_EDGE_FALLING; // Left
  //pdmConfig.edge               = (nrf_pdm_edge_t)PDM_EDGE_RISING; // Right
  pdmConfig.interrupt_priority = NRFX_PDM_CONFIG_IRQ_PRIORITY;
  pdmConfig.pin_clk            = PDM_CLK_PIN;
  pdmConfig.pin_din            = PDM_DATA_PIN;
  
  if(params){
    if(params->mode != 0xFF){
    #if AUDIO_PARAMS_DEBUG
      NRF_LOG_RAW_INFO("%08d [audio] Updating mode to: %d\n", systemTimeGetMs(), params->mode);
    #endif
      pdmConfig.mode               = params->mode;
    }
    
    if(params->div_clk != 0xFF){
    #if AUDIO_PARAMS_DEBUG
      NRF_LOG_RAW_INFO("%08d [audio] Updating clock freq to: 0x%x\n", systemTimeGetMs(), params->div_clk);
    #endif
      pdmConfig.clock_freq         = (nrf_pdm_freq_t) params->div_clk;
    }

    if(params->left_gain != 0xFF){
    #if AUDIO_PARAMS_DEBUG
      NRF_LOG_RAW_INFO("%08d [audio] Updating left gain to: %d\n", systemTimeGetMs(), params->left_gain);
      #endif
      pdmConfig.gain_l             = params->left_gain; // 3dB gain
    }
        
    if(params->right_gain != 0xFF){
    #if AUDIO_PARAMS_DEBUG
      NRF_LOG_RAW_INFO("%08d [audio] Updating right gain to: %d\n", systemTimeGetMs(), params->right_gain);
    #endif
      pdmConfig.gain_r             = params->right_gain; // 3dB gain
    }

    if(params->decimation_factor != 0xFF){
    #if AUDIO_PARAMS_DEBUG
    NRF_LOG_RAW_INFO("%08d [audio] Updating decimation factor to: %d\n", systemTimeGetMs(), params->decimation_factor);
    #endif
      m_decimation_factor = params->decimation_factor;
    }

    if(params->ratio != 0xFF){
      m_pdm_ratio = params->ratio;
    }

    if(params->use_compression != 0xFF){
    #if AUDIO_PARAMS_DEBUG
    NRF_LOG_RAW_INFO("%08d [audio] Updating use compression to: %d\n", systemTimeGetMs(), params->use_compression);
    #endif
      m_use_compression = params->use_compression > 0;
    }

    if(m_use_compression){
      if(params->compression_bitrate >= 512 && params->compression_bitrate <= 512000){
        #if AUDIO_PARAMS_DEBUG
        NRF_LOG_RAW_INFO("%08d [audio] Updating compression rate to to: %d\n", systemTimeGetMs(), params->compression_bitrate);
        #endif
        m_compression_bitrate = params->compression_bitrate;
      }else{
        #if AUDIO_PARAMS_DEBUG
        NRF_LOG_RAW_INFO("%08d [audio] Compression rate %d unavailable, defaulting to: %d\n", systemTimeGetMs(), params->compression_bitrate, OPUS_BITRATE);
        #endif

        m_compression_bitrate = OPUS_BITRATE;
      }
    }
  }
  
  current_params = pdmConfig;
}

//void audioStart(void){
//  if (!streamStarted) {
//    nrfx_err_t errorStatus;
//    streamStarted = true;
//    errorStatus = nrfx_pdm_start();
//    ASSERT(errorStatus == NRFX_SUCCESS);
//    NRF_LOG_RAW_INFO("%08d [audio] pdm start\n", systemTimeGetMs());
//  }
//}

void audioStop(void){
  streamStarted = false;
  nrfx_pdm_stop();
}

void audioDeInit(void){  
  nrf_gpio_pin_clear(MIC_EN_PIN);
  
  if(streamStarted) {
    audioStop();
    nrfx_pdm_uninit();
  }
  
  NRF_LOG_RAW_INFO("%08d [audio] deinitialized\n", systemTimeGetMs());
}

mic_mode_t audio_get_mode(void){
  return current_mic_mode;
}

void audio_set_mode(mic_mode_t mode){
  current_mic_mode = mode;
}

void audio_set_next_recording_beginning(int64_t time_ms){
  audio_record_time_start = time_ms;

  audio_record_sample_start = channels * audio_record_time_start * current_freq / 1000;
  audio_record_sample_end = audio_record_sample_start + channels * round(1 + RECORDING_DURATION_MS * current_freq / 1000);

#if RECORD_TASK_DEBUG
  NRF_LOG_RAW_INFO("Next recording will start at %lld samples\n", audio_record_sample_start);
  NRF_LOG_RAW_INFO("Next recording will end at %lld samples\n", audio_record_sample_end);
#endif

  m_audio_recording_finished = false;
  m_ts_disabled = true;
}

bool save_recording(int64_t t1, uint16_t data_length, uint8_t **buffer, uint16_t *adjusted_length){
  t1 += pdmDroppedCount;
  int64_t t2 = t1 + data_length / 2;

  //NRF_LOG_RAW_INFO("Audio buffer sample start: %lld\n", t1);
  //NRF_LOG_RAW_INFO("Audio buffer sample end: %lld\n", t2);

  //NRF_LOG_RAW_INFO("audio_record_sample_start: %lld\n", audio_record_sample_start);
  //NRF_LOG_RAW_INFO("audio_record_sample_end: %lld\n", audio_record_sample_end);

  if((t1 <= audio_record_sample_start && t2 >= audio_record_sample_start)
      || (t1 >= audio_record_sample_start && t2 < audio_record_sample_end)
      || (t1 < audio_record_sample_end && t2 >= audio_record_sample_end)){
      int64_t start_sample = max(audio_record_sample_start, t1);
      int64_t end_sample = min(audio_record_sample_end, t2);
            
      *buffer = *buffer + sizeof(int16_t) * (start_sample - t1);
      *adjusted_length = sizeof(int16_t) * (end_sample - start_sample);

      //NRF_LOG_RAW_INFO("Audio buffer sample start: %lld\n", t1);
      //NRF_LOG_RAW_INFO("Audio buffer sample end: %lld\n", t2);
      
      //NRF_LOG_RAW_INFO("Buffer adjusted length\n", *adjusted_length);

      return true;
  }
  
  return false;
}

bool audio_recording_finished(int64_t n_samples){
  n_samples += pdmDroppedCount;
  if(audio_record_sample_end > 0 && (audio_record_sample_end <= n_samples) && !m_audio_recording_finished){
    //NRF_LOG_RAW_INFO("Dropped packets: %lld\n", pdmDroppedCount);
    m_audio_recording_finished = true;
    m_ts_disabled = false;
    return true;
  }
  return false;
}

bool audio_compression_enabled(void){
  return m_use_compression;
}