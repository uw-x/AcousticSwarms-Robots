#include "ble_cus.h"
#include "ble_sss.h"
#include "nrf_log.h"

#include "sensors.h"
#include "event.h"
#include "fuel_gauge.h"
#include "accel.h"
#include "gyro.h"
#include "ir.h"


static sensor_t *m_sensors[NUM_SENSORS];

static uint32_t ble_sss_control_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t control_params;
  memset(&control_params, 0, sizeof(control_params));

  control_params.uuid              = SSS_CONTROL_CHAR_UUID;
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
                                &p_cus->characteristic_handles[SSS_CONTROL_CHAR_HANDLE]);
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

static uint32_t ble_sss_data_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t params;
  memset(&params, 0, sizeof(params));

  params.uuid              = SSS_SENSOR_DATA_UUID;
  params.uuid_type         = p_cus->uuid_type;
  params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  params.char_props.notify = 1;
  params.char_props.indicate = 1;
  params.char_props.write_wo_resp = 1; // to enable writing
  params.cccd_write_access = SEC_OPEN;
  params.is_var_len        = 1;

  err_code = characteristic_add(p_cus->service_handle,
                                &params,
                                &p_cus->characteristic_handles[SSS_SENSOR_DATA_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

uint32_t ble_sss_init(ble_cus_t * p_cus){  
  uint32_t   err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t base_uuid = {SENSOR_STREAMING_SERVICE_UUID_BASE}; // Add Custom Service UUID

  // Initialize service structure
  p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

  err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = SENSOR_STREAMING_SERVICE_UUID;

  // Add the Device Control Service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
  VERIFY_SUCCESS(err_code);

  p_cus->characteristic_handles = malloc(sizeof(ble_gatts_char_handles_t) * SENSOR_STREAMING_SERVICE_NUM_CHARS);

  // Add Control Characteristic
  ble_sss_control_char_add(p_cus);

  // Add Data Characteristic
  ble_sss_data_char_add(p_cus);

  // TODO: Find a better place to put this, and reset this
  p_cus->kbytes_sent = 0;
  p_cus->bytes_sent = 0;

  m_sensors[SENSOR_ACCEL_ID] = accel_get_sensor_class();
  m_sensors[SENSOR_FG_ID] = fg_get_sensor_class();
  //m_sensors[SENSOR_MAG_ID] = mag_get_sensor_class();
  m_sensors[SENSOR_GYRO_ID] = gyro_get_sensor_class();

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

  //NRF_LOG_RAW_INFO("Char Handle: %d\n", p_evt_write->handle);
  //NRF_LOG_RAW_INFO("Control Handle: %d\n", p_cus->characteristic_handles[SSS_CONTROL_CHAR_HANDLE].value_handle);
  
  const uint8_t *data = p_evt_write->data;
  // Control characteristic
  if (p_evt_write->handle == p_cus->characteristic_handles[SSS_CONTROL_CHAR_HANDLE].value_handle) {
    //NRF_LOG_RAW_INFO("Data: %d\n", data[0]);
    if (data[0] == SSS_STREAM_ENABLE || data[0] == SSS_STREAM_DISABLE){
      uint8_t mask = data[1];
      //NRF_LOG_RAW_INFO("Enable: %d\n", data[1]);
      for(uint8_t i = 0; i < 8; i++){
        if(mask & (1 << i)){
          if(data[0] == SSS_STREAM_ENABLE){
            m_sensors[i]->stream_enabled = 1;
             m_sensors[i]->cfg_handler(data, p_evt_write->len);
          }else{
            m_sensors[i]->stream_enabled = 0;
          }
        }
      }
    }else if(data[0] == SSS_SENSOR_PARAMS_CONFIGURE){
      m_sensors[data[1]]->cfg_handler(data, p_evt_write->len);
    }else if(data[0] == SSS_CALIBRATE_ALL){
      for(uint8_t i = 0; i < NUM_SENSORS; i++){
        if(m_sensors[i] && m_sensors[i]->calibrate){
          NRF_LOG_RAW_INFO("Calibrating sensor: %d\n", i);
          m_sensors[i]->calibrate();
        }
      }
    }else if(data[0] == SSS_READ_FUEL_GAUGE){
      eventQueuePush(EVENT_FUEL_GAUGE_INTERRUPT);
    }else if(data[0] == SSS_ENABLE_COLLISION_DETECTION){
      bool is_moving = data[1];
      if(is_moving){
        eventQueuePush(EVENT_ENABLE_MOVING_COLLISION_DETECTION);
      }else{
        eventQueuePush(EVENT_ENABLE_STATIONARY_COLLISION_DETECTION);
      }
    }else if(data[0] == SSS_DISABLE_COLLISION_DETECTION){
      eventQueuePush(EVENT_DISABLE_COLLISION_DETECTION);
    }
    else if(data[0] == SSS_ENABLE_IR_STATUS_STREAM){
      //NRF_LOG_RAW_INFO("%08d [ble_sss] ENABLED IR STATUS STREAM\n", systemTimeGetMs());
      if(!ir_enabled()) ir_enable(true);
      ir_enable_stream(true);
    }
    else if(data[0] == SSS_DISABLE_IR_STATUS_STREAM){
      if(ir_enabled()) ir_disable();
      ir_enable_stream(false);
    }else if(data[0] == SSS_IR_STATUS_READ){
      if(!ir_enabled(false)) ir_enable(false);
      ir_update_status();
      eventQueuePush(EVENT_IR_STATUS_UPDATED);
    }else if(data[0] == SSS_ENABLE_EDGE_DETECTION){
      ir_set_mode(IR_MODE_EDGE_DETECTION);
      ir_enable(false);
    }else if(data[0] == SSS_DISABLE_EDGE_DETECTION){
      if(ir_get_mode() == IR_MODE_EDGE_DETECTION){
        ir_set_mode(IR_NONE);
        if(!ir_streaming_enabled()){
          ir_disable();
        }
      }
    }
  }
}

void ble_sss_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context){
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

bool ble_sss_notify_sensor_data(ble_cus_t * p_cus, const ble_gatts_char_handles_t * const handles, sensor_t *sensor){
  if (p_cus == NULL) { return NRF_ERROR_NULL; }
  static uint8_t data[256];
  data[0] = sensor->id;
  memcpy_fast(data+1, sensor->sensor_data, sensor->data_length);
  uint16_t length = sensor->data_length + 1;

  ble_gatts_hvx_params_t const hvx_param = {
    .handle = handles->value_handle,
    .p_data = data,
    .p_len  = &length,
    .type   = BLE_GATT_HVX_NOTIFICATION,
  };

  uint32_t err_code = NRF_SUCCESS;

  err_code = sd_ble_gatts_hvx(p_cus->conn_handle, &hvx_param);

  //NRF_LOG_RAW_INFO("conn_handle: %d, value_handle: %d\n", p_cus->conn_handle, handles->value_handle);

  if (err_code == NRF_ERROR_RESOURCES) {
    //NRF_LOG_RAW_INFO("sd_ble_gatts_hvx() failed: resource busy\n", err_code);
    return false; // return busy flag
  } else if (err_code != NRF_SUCCESS) {
    //NRF_LOG_ERROR("sd_ble_gatts_hvx() failed: 0x%x", err_code);
    return false;
  }

  return true;

}
