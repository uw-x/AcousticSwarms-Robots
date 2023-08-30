#include "ble_cus.h"
#include "ble_ass.h"

#include "sdk_common.h"
#include "ble_srv_common.h"
//#include "boards.h"
#include "nrf_log.h"
#include "event.h"
#include "timers.h"
#include "audio.h"

//#include "gpio.h"

//#include "nrf_gpio.h"

#include <string.h>

static uint32_t ble_ass_control_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t control_params;
  memset(&control_params, 0, sizeof(control_params));

  control_params.uuid              = ASS_CONTROL_CHAR_UUID;
  control_params.uuid_type         = p_cus->uuid_type;
  control_params.max_len           = 32;
  control_params.init_len          = 1;
  control_params.char_props.notify = 0;
  control_params.char_props.write_wo_resp = 1;
  control_params.char_props.read   = 0;
  control_params.write_access      = SEC_OPEN;
  control_params.cccd_write_access = SEC_OPEN;
  control_params.is_var_len        = 0;

  err_code = characteristic_add(p_cus->service_handle, 
                                &control_params,
                                &p_cus->characteristic_handles[ASS_CONTROL_CHAR_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

static uint32_t ble_ass_status_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t status_params;
  memset(&status_params, 0, sizeof(status_params));

  status_params.uuid              = ASS_STATUS_CHAR_UUID;
  status_params.uuid_type         = p_cus->uuid_type;
  status_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  status_params.char_props.notify = 1;
  status_params.char_props.indicate = 1;
  status_params.char_props.read   = 1;
  status_params.read_access       = SEC_OPEN;
  status_params.cccd_write_access = SEC_OPEN;
  status_params.is_var_len        = 1;


  err_code = characteristic_add(p_cus->service_handle, 
                                &status_params, 
                                &p_cus->characteristic_handles[ASS_STATUS_CHAR_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

static uint32_t ble_ass_mic_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t mic_params;
  memset(&mic_params, 0, sizeof(mic_params));

  mic_params.uuid              = ASS_MIC_CHAR_UUID;
  mic_params.uuid_type         = p_cus->uuid_type;
  mic_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  mic_params.char_props.notify = 1;
  mic_params.char_props.indicate = 1;
  // mic_params.char_props.write_wo_resp = 1; // to enable writing
  // mic_params.char_props.read = 1;          // to enable reading
  mic_params.cccd_write_access = SEC_OPEN;
  mic_params.is_var_len        = 1;

  err_code = characteristic_add(p_cus->service_handle, 
                                &mic_params,
                                &p_cus->characteristic_handles[ASS_MIC_CHAR_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

static uint32_t ble_ass_spk_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t spk_params;
  memset(&spk_params, 0, sizeof(spk_params));

  spk_params.uuid              = ASS_SPK_CHAR_UUID;
  spk_params.uuid_type         = p_cus->uuid_type;
  spk_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  spk_params.char_props.notify = 1;
  spk_params.char_props.indicate = 1;
  spk_params.char_props.write_wo_resp = 1; // to enable writing
  spk_params.cccd_write_access = SEC_OPEN;
  spk_params.is_var_len        = 1;

  err_code = characteristic_add(p_cus->service_handle,
                                &spk_params,
                                &p_cus->characteristic_handles[ASS_SPK_CHAR_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

uint32_t ble_ass_init(ble_cus_t * p_cus){  
  // IF NOTHING WORKS SEE THIS  
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm
  
  uint32_t   err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t base_uuid = {AUDIO_STREAMING_SERVICE_UUID_BASE}; // Add Custom Service UUID

  // Initialize service structure
  p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

  err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = AUDIO_STREAMING_SERVICE_UUID;

  // Add the Device Control Service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
  VERIFY_SUCCESS(err_code);

  p_cus->characteristic_handles = malloc(sizeof(ble_gatts_char_handles_t) * AUDIO_STREAMING_SERVICE_NUM_CHARS);

  // Add Control Characteristic
  ble_ass_control_char_add(p_cus);

  // Add Status Characteristic
  ble_ass_status_char_add(p_cus);

  // Add Microphone Characteristic
  ble_ass_mic_char_add(p_cus);

  // Add Speaker Characteristic
  ble_ass_spk_char_add(p_cus);

  // TODO: Find a better place to put this, and reset this
  p_cus->kbytes_sent = 0;
  p_cus->bytes_sent = 0;

  return NRF_SUCCESS;
}

static void on_connect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt){
  p_cus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
  //NRF_LOG_RAW_INFO("Connected! on_connect\n");
}

static void on_disconnect(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt){
  UNUSED_PARAMETER(p_ble_evt);
  p_cus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_cus_t * p_cus, ble_evt_t const * p_ble_evt){  
  const ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

  // Mic characteristic
  // Check if the handle passed with the event matches the Mic Value Characteristic handle.
  if (p_evt_write->handle == p_cus->characteristic_handles[ASS_MIC_CHAR_HANDLE].value_handle) {
    NRF_LOG_INFO("Today's Menu: Shio Ramen");
  }

  //NRF_LOG_RAW_INFO("Control Handle: %d\n", p_cus->characteristic_handles[ASS_CONTROL_CHAR_HANDLE].value_handle);

  // Check if the Mic value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
  if ((p_evt_write->handle == p_cus->characteristic_handles[ASS_MIC_CHAR_HANDLE].cccd_handle) && (p_evt_write->len == 2)){
    NRF_LOG_INFO("Received CCCD change to %d", uint16_decode(p_evt_write->data));
    // CCCD written
    if(ble_srv_is_notification_enabled(p_evt_write->data)) {
      //NRF_LOG_INFO("Mic characteristic notification enabled");
      eventQueuePush(EVENT_AUDIO_MIC_NOTIFICATIONS_ENABLED);
    }else {
      // End mic stream signal
      //NRF_LOG_INFO("Mic characteristic notification disabled");
      eventQueuePush(EVENT_AUDIO_MIC_NOTIFICATIONS_DISABLED);
    }
  }
  const uint8_t *data = p_evt_write->data;
  // Control characteristic
  if (p_evt_write->handle == p_cus->characteristic_handles[ASS_CONTROL_CHAR_HANDLE].value_handle) {
    //NRF_LOG_RAW_INFO("Data: %d\n", data[0]);
    if (data[0] == ASS_CONTROL_ENABLE_MASTER) {
      eventQueuePush(EVENT_TIMESYNC_MASTER_ENABLE);
    } else if (data[0] == ASS_CONTROL_ENABLE_SLAVE) {
      eventQueuePush(EVENT_TIMESYNC_SLAVE_ENABLE);
    }else if(data[0] == ASS_CONTROL_TS_DISABLE){
      eventQueuePush(EVENT_TIMESYNC_DISABLE);
    }
    //else if (p_evt_write->data[0] == ASS_CONTROL_MIC_STREAM_STOP) {
    //  eventQueuePush(EVENT_BLE_DATA_STREAM_STOP);
    //}
    else if (p_evt_write->data[0] == ASS_CONTROL_MIC_MODE_SET_STREAM) {
      eventQueuePush(EVENT_BLE_DATA_STREAM_START);
    }
    else if (p_evt_write->data[0] == ASS_CONTROL_MIC_MODE_SET_RECORD) {
      eventQueuePush(EVENT_BLE_DATA_RECORD_START);
    }
    else if (data[0] == ASS_CONTROL_START_CLICK) {
      eventQueuePush(EVENT_AUDIO_START_CLICK);
    }
    else if (data[0] == ASS_CONTROL_RECORDING_SEND) {
      eventQueuePush(EVENT_RECORDING_REQUESTED);
    }
    else if(data[0] == ASS_CONTROL_MIC_UPDATE_PARAMS){
      NRF_LOG_INFO("Params updated!");
      audio_params_t params;
      params.div_clk = (data[1] << 24) + (data[2] << 16) + (data[3] << 8) + data[4];
      params.decimation_factor = data[5];
      params.left_gain = data[6];
      params.right_gain = data[7];
      params.mode = data[8];
      params.ratio = data[9];
      params.use_compression = data[10];
      params.compression_bitrate = (data[11] << 24) + (data[12] << 16) + (data[13] << 8) + data[14];
      audio_params_update(&params);
    }else{
      NRF_LOG_INFO("Unhandled write event: %d!", data[0]);
    }
  }

  if ((p_evt_write->handle == p_cus->characteristic_handles[ASS_CONTROL_CHAR_HANDLE].cccd_handle) && (p_evt_write->len == 2)) {
    NRF_LOG_INFO("Control characteristic notification enabled");
  }
}

void ble_ass_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context){
  ble_cus_t * p_cus = (ble_cus_t *) p_context;
  if (p_cus == NULL || p_ble_evt == NULL) { return; }

  switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
      on_connect(p_cus, p_ble_evt);
      break;

    case BLE_GAP_EVT_DISCONNECTED:
      on_disconnect(p_cus, p_ble_evt);
      break;

    case BLE_GATTS_EVT_WRITE:
      on_write(p_cus, p_ble_evt);
      break;

    default:
      // No implementation needed.
      break;
  }
}
