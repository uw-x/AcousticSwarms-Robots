#include "recorder.h"
#include "ringbuffer.h"
#include "ble_manager.h"
#include "ble_config.h"
#include <stdlib.h>
#include "arm_const_structs.h"

#include "nrf_log.h"
#include "timers.h"
#include "event.h"
#include "app_error.h"

static uint8_t packet[PACKET_LENGTH+2];


void recorder_init(recording_t* rec, uint8_t *buf, uint32_t size){
  rec->data = buf;

  if(rec->metadata == NULL){
    rec->metadata = malloc(RECORDER_MAX_DATA_PACKETS * sizeof(packet_metadata_t));
  }
  
  rec->num_packets = 0;
  rec->packets_sent = 0;
  rec->samples_sent = 0;
  rec->requested = false;
  rec->length = 0;
}

void recorder_deinterleave_channel(recording_t* rec, int16_t* buff, bool right_channel){
  APP_ERROR_CHECK_BOOL(rec->length % 4 == 0);
    
  // iterate 4-shorts at a time
  uint16_t srcABLength_4 = (rec->length >> 2);
  
  typedef union{
      uint64_t wide;
      struct { int16_t a1; int16_t b1; int16_t a2; int16_t b2; } narrow;
  } ab16x4_t;

  uint64_t *srcAB64 = (uint64_t *) rec->data;
  int j = 0, i = 0;
  ab16x4_t cursor;
  for (; i < srcABLength_4; i++){
      cursor.wide = srcAB64[i];
      if(!right_channel){
        buff[j++  ] = cursor.narrow.a1;
        buff[j++  ] = cursor.narrow.a2;
      }else{
        buff[j++] = cursor.narrow.b1;
        buff[j++] = cursor.narrow.b2;
      }
  }
  return;
}

bool recorder_add(recording_t* rec, uint8_t *data, uint8_t size){
  // Record length & sequence number
  rec->metadata[rec->num_packets].length = size;
  rec->metadata[rec->num_packets].sequence_number = rec->num_packets + 1;
  rec->num_packets++;

  // Record data
  memcpy_fast(rec->data + sizeof(int16_t) * rec->length, data, size);
  rec->length += size / 2;
  //NRF_LOG_RAW_INFO("Length: %d\n", rec->length);
  
  return true;
}

bool recorder_available(recording_t* rec){
  //NRF_LOG_RAW_INFO("[record] SAMPLES SENT %d REC LENGTH %d\n", rec->samples_sent, rec->length);
  //if(ble_cus_indication_in_progress()){
  //  NRF_LOG_INFO("[recorder_send_all]  ***** INDICATION IN PROGRESS *****");
  //}else{
  //  NRF_LOG_INFO("[recorder_send_all]  ***===* INDICATION IN PROGRESS -=======*");
  //}
  return rec->samples_sent < rec->length;
}

// Tries to send next packet
bool recorder_send_packet(recording_t* rec){
  uint16_t i;

  //if(ble_cus_indication_in_progress()){
  //    NRF_LOG_INFO("[BEFORE]  ***** INDICATION IN PROGRESS *****");
  //  }else{
  //    NRF_LOG_INFO("[BEFORE]  ***===* INDICATION IN PROGRESS -=======*");
  //  }

  uint8_t length = rec->metadata[rec->packets_sent].length;
  packet[0] = (rec->metadata[rec->packets_sent].sequence_number >> 8) & 0xFF;
  packet[1] = rec->metadata[rec->packets_sent].sequence_number & 0xFF;

  //NRF_LOG_RAW_INFO("LENGTH: %d\n", length);
  //NRF_LOG_RAW_INFO("Sequence number: %d\n",  rec->metadata[rec->packets_sent].sequence_number);

  memcpy_fast(packet + 2, rec->data + rec->samples_sent * 2, length);
  //memccpy();
  //memcpy();

  if(ble_manager_send_mic_data(packet, length + 2, 0)){
    //NRF_LOG_RAW_INFO("Sent %d bytes, sequence number %d.\n", length + 2, (packet[0] << 8) + packet[1]);
    rec->samples_sent += length/2;
    rec->packets_sent ++;
    //NRF_LOG_RAW_INFO("Samples sent: %d\n", rec->samples_sent);

    //if(ble_cus_indication_in_progress()){
    //  NRF_LOG_INFO("[AFTER]  ***** INDICATION IN PROGRESS *****");
    //}else{
    //  NRF_LOG_INFO("[AFTER]  ***===* INDICATION IN PROGRESS -=======*");
    //}

    return true;
  }
  
  return false;
}

bool recorder_send_all(recording_t* rec){
  // Send at most 4 packets so as not to starve other events.
  uint8_t i = 0;
  while(recorder_available(rec) && recorder_send_packet(rec) && i++ < 4);
  
  //NRF_LOG_RAW_INFO("Sent %d samples.\n", rec->samples_sent);
  if(!recorder_available(rec)) {
    NRF_LOG_RAW_INFO("%08d Finished. Sent %d samples.\n", systemTimeGetUs(), rec->samples_sent);
    eventQueuePush(EVENT_RECORDING_SEND_FINISHED);
    //NRF_LOG_RAW_INFO("%08d QUEUED RECORD FINISHED.\n", systemTimeGetUs());
    rec->requested = false;
  }
  return !recorder_available(rec);
}

void recorder_reset(recording_t *rec){
  rec->num_packets = 0;
  rec->packets_sent = 0;
  rec->samples_sent = 0;
  rec->length = 0;
}
