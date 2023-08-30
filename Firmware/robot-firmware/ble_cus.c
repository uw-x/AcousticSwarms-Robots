#include "ble_cus.h"

#include <stdint.h>
#include <stdbool.h>

#include "sdk_common.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "nrf_log.h"
#include "nrf_atomic.h"

#include "timers.h"
#include "event.h"


#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

//static volatile bool indication_in_progress = false;

static nrf_atomic_flag_t indication_in_progress;

void ble_cus_indication_confirm(void){
  nrf_atomic_flag_clear(&indication_in_progress);
}

bool ble_cus_indication_in_progress(void){
  return indication_in_progress;
}

void ble_cus_on_gatt_evt(ble_cus_t * p_cus, nrf_ble_gatt_evt_t const * p_gatt_evt){
  if (p_gatt_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    p_cus->max_payload_len = p_gatt_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
  }
}

bool ble_cus_send_hvx(ble_cus_t * p_cus, const ble_gatts_char_handles_t *const handles, uint8_t * data, uint16_t length, bool use_indication){
  uint16_t payload_len = p_cus->max_payload_len;
  ble_cus_evt_t evt;

  if (p_cus == NULL) { return NRF_ERROR_NULL; }
  
  if(length > payload_len) { 
    NRF_LOG_INFO("length > max(%d), truncating", p_cus->max_payload_len); 
  }else{
    payload_len = length; 
  }

  ble_gatts_hvx_params_t const hvx_param =
  {
    .handle = handles->value_handle,
    .p_data = data,
    .p_len  = &payload_len,
    .type   = (use_indication ? BLE_GATT_HVX_INDICATION : BLE_GATT_HVX_NOTIFICATION),
  };

  uint32_t err_code = NRF_SUCCESS;

  err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_param);

  //NRF_LOG_RAW_INFO("conn_handle: %d, value_handle: %d\n", p_cus->conn_handle, handles->value_handle);

  if (err_code == NRF_ERROR_RESOURCES) {
    //NRF_LOG_RAW_INFO("sd_ble_gatts_hvx() failed: resource busy\n", err_code);
    return false; // return busy flag
  } else if (err_code != NRF_SUCCESS) {
    NRF_LOG_ERROR("sd_ble_gatts_hvx() failed: 0x%x", err_code);
    return false;
  }

  p_cus->bytes_sent += payload_len;

  if (p_cus->kbytes_sent != (p_cus->bytes_sent / 1024)) {
    p_cus->kbytes_sent = (p_cus->bytes_sent / 1024);
    evt.evt_type             = BLE_CUS_EVT_TRANSFER_1KB;
    evt.bytes_transfered_cnt = p_cus->bytes_sent;
    if(( (evt.bytes_transfered_cnt / 1024) % 20) == 0) {
      NRF_LOG_RAW_INFO("%08d [ble] sent %ukB\n", systemTimeGetMs(), (evt.bytes_transfered_cnt / 1024));
    }
  }

  return true;
}


bool ble_cus_send_notification(ble_cus_t * p_cus, const ble_gatts_char_handles_t *const handles, uint8_t * data, uint16_t length){
  //NRF_LOG_RAW_INFO("CONN HANDLE: %d, VALUE HANDLE: %d\n", p_cus->conn_handle, handles->value_handle);
  return ble_cus_send_hvx(p_cus, handles, data, length, false);
}

bool ble_cus_send_indication(ble_cus_t *p_cus, const ble_gatts_char_handles_t * const handles, uint8_t * data, uint16_t length){
  if(!indication_in_progress){
    nrf_atomic_flag_set(&indication_in_progress);
    if(ble_cus_send_hvx(p_cus, handles, data, length, true)){
      //NRF_LOG_INFO("[ble_cus] SENDING HVX");
      return true;
    }else{
      nrf_atomic_flag_clear(&indication_in_progress);
      //NRF_LOG_INFO("[ble_cus] FAILED TO SEND INDICATION\n");
      return false;
    }
  }
  NRF_LOG_INFO("[ble_cus] INDICATION IN PROGRESS\n");
  return false;
}
