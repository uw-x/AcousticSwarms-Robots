#ifndef __RECORDER_H__
#define __RECORDER_H__

#include "ringbuffer.h"

#define RECORDER_MAX_DATA_PACKETS 256

typedef struct{
  uint8_t length;
  uint16_t sequence_number;
} packet_metadata_t;

typedef struct{
  uint8_t *data;
  packet_metadata_t *metadata;
  uint16_t num_packets;
  uint16_t packets_sent;
  uint64_t samples_sent;
  uint32_t length; // Length of data in SAMPLES
  bool requested;
  
  uint64_t start_time;
  uint64_t end_time;
} recording_t;

void recorder_create(recording_t* rec, uint32_t size);
void recorder_init(recording_t* rec, uint8_t *buf, uint32_t size);
bool recorder_add(recording_t* rec, uint8_t *data, uint8_t size);
bool recorder_available(recording_t* rec);
void recorder_deinterleave_channel(recording_t* rec, int16_t* buff, bool right_channel);
//void recording_cvt_int16_float32(recording_t* rec);
//void recording_strip_metadata(recording_t* rec);

// Returns true if packet is successfully sent
bool recorder_send(recording_t* rec);

// Returns true if all data is successfully sent
bool recorder_send_all(recording_t* rec);

void recorder_reset(recording_t *rec);

#endif
