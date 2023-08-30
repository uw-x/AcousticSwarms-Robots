#ifndef _AUDIO_H_
#define _AUDIO_H_

#include <stdbool.h>

// audio.h

typedef  enum {
  AUDIO_MODE_NONE = 0,
  AUDIO_MODE_RECORDING,
  AUDIO_MODE_STREAMING,
} mic_mode_t;

void audioInit(void);
void audioDeInit(void);
//void audioStart(void);
void audioStop(void);
uint8_t audioGetMicData(uint8_t **data);
void audioUpdateTicksAhead(void);
bool audioStreamStarted(void);
void audio_linear_interpolate(int16_t *buffer, int16_t l1, int16_t l2);
void audioSetStreamStarted(bool);
uint32_t audioGetPdmStartTaskAddress(void);
uint32_t audioGetPdmBufferCount(void);
uint64_t audioGetPdmSampleCount(void);
uint16_t audio_buffer_length(void);
void audio_ready(void);
mic_mode_t audio_get_mode(void);
void audio_set_mode(mic_mode_t mode);
void audio_reset(void);
// Sets the recording bounds for the next recording.
void audio_set_next_recording_beginning(int64_t time_ms);
bool audio_recording_finished(int64_t n_samples);
void audio_resample_50_48(int16_t *buffer, uint16_t S1, uint16_t S2);
uint8_t  audio_get_channels(void);
bool audio_compression_enabled(void);

bool audio_encode_data(uint8_t *data, uint16_t length, uint8_t **enc, uint16_t *enc_length);

// Check whether the current buffer should be saved to the recording & clips the buffer to fit pre-computed recording bounds
bool save_recording(int64_t current_buffer_sample_count, uint16_t data_length, uint8_t **buffer, uint16_t *adjusted_length);
void audio_resample(int16_t *buffer, uint16_t S1, uint16_t S2, int channels);

typedef struct{
  uint32_t div_clk;
  uint8_t decimation_factor;
  uint8_t left_gain;
  uint8_t right_gain;
  uint8_t mode;
  uint8_t use_compression;
  uint32_t compression_bitrate;
  uint8_t ratio;
} audio_params_t;

void audio_params_update(audio_params_t *params);

#endif