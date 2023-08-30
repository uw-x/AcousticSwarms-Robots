#include "ble_cus.h"
#include "ble_dcs.h"

#include "sdk_common.h"
#include "ble_srv_common.h"

#include "nrf_log.h"
#include "event.h"
#include "timers.h"
#include "ir.h"
#include "motors.h"

#include "ts_tasks.h"
#include "nav.h"
#include "charger.h"
#include "profile.h"

#include <string.h>

static uint32_t ble_dcs_command_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t control_params;
  memset(&control_params, 0, sizeof(control_params));

  control_params.uuid              = DCS_COMMAND_CHAR_UUID;
  control_params.uuid_type         = p_cus->uuid_type;
  control_params.max_len           = NRF_SDH_BLE_GATT_MAX_MTU_SIZE;
  control_params.init_len          = 1;
  control_params.char_props.notify = 0;
  control_params.char_props.write_wo_resp = 1;
  control_params.char_props.read   = 0;
  control_params.write_access      = SEC_OPEN;
  control_params.cccd_write_access = SEC_OPEN;
  control_params.is_var_len        = 0;

  err_code = characteristic_add(p_cus->service_handle, 
                                &control_params,
                                &p_cus->characteristic_handles[DCS_COMMAND_CHAR_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

static uint32_t ble_dcs_status_char_add(ble_cus_t * p_cus){
  uint32_t err_code;
  ble_add_char_params_t status_params;
  memset(&status_params, 0, sizeof(status_params));

  status_params.uuid              = DCS_STATUS_CHAR_UUID;
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
                                &p_cus->characteristic_handles[DCS_STATUS_CHAR_HANDLE]
                                );
  APP_ERROR_CHECK(err_code);

  return NRF_SUCCESS;
}

uint32_t ble_dcs_init(ble_cus_t * p_cus){  
  // IF NOTHING WORKS SEE THIS  
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
  //BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm
  
  uint32_t   err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t base_uuid = {DEVICE_CONTROL_SERVICE_UUID_BASE}; // Add Custom Service UUID

  // Initialize service structure
  p_cus->conn_handle               = BLE_CONN_HANDLE_INVALID;

  err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_cus->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_cus->uuid_type;
  ble_uuid.uuid = DEVICE_CONTROL_SERVICE_UUID;

  // Add the Device Control Service
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cus->service_handle);
  VERIFY_SUCCESS(err_code);

  p_cus->characteristic_handles = malloc(sizeof(ble_gatts_char_handles_t) * DEVICE_CONTROL_SERVICE_NUM_CHARS);

  // Add Command Characteristic
  ble_dcs_command_char_add(p_cus);

  // Add Status Characteristic
  ble_dcs_status_char_add(p_cus);

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
  
  if(p_evt_write->handle == p_cus->characteristic_handles[DCS_COMMAND_CHAR_HANDLE].value_handle){
      const uint8_t *data = p_evt_write->data;
      //NRF_LOG_RAW_INFO("WRITE %x\n", data[0]);
      switch(data[0]){
        case DCS_COMMAND_SYNC_TASK_CREATE:{
          uint8_t task_id = data[1];
          uint32_t arm_interval = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];
          ts_sync_task_add(task_id, arm_interval);
        }
        break;
        case DCS_COMMAND_SYNC_TASK_CREATE_RECURRING:{
          uint8_t task_id = data[1];
          uint32_t period = (data[2] << 24) + (data[3] << 16) + (data[4] << 8) + data[5];
          uint32_t start_interval = (data[6] << 24) + (data[7] << 16) + (data[8] << 8) + data[9];
          uint32_t end_interval = (data[10] << 24) + (data[11] << 16) + (data[12] << 8) + data[13];
          if(end_interval == 0){
            end_interval = 1LL << 31;
          }
          ts_sync_task_add_recurring(task_id, period, start_interval, end_interval);
        }
        break;
        case DCS_COMMAND_ROBOT_MOTION:{
          motion_code_t code = data[1];
          //NRF_LOG_RAW_INFO("Motion command %d\n", code);
          // Update motors
          nav_motion_update(code, data + 2, p_evt_write->len - 2);
        }
        break;
        case DCS_COMMAND_ROBOT_POSITION_UPDATE:{
          float* values = (float*) (data + 1);
          nav_state_t current = {0}, m1 = {0}, m2 = {0};
          
          //NRF_LOG_RAW_INFO("Length: %d\n", p_evt_write->len);
          //for(int i = 1; i < p_evt_write->len; i++){
          //  NRF_LOG_RAW_INFO("0x%x ", data[i]);
          //}NRF_LOG_RAW_INFO("\n"); 

          //for(int i = 0; i < 9; i++){
          //  float k = ((float*) (data+1))[i];
          //  NRF_LOG_RAW_INFO("%d | " NRF_LOG_FLOAT_MARKER " ", i, NRF_LOG_FLOAT(k));
            
          //}NRF_LOG_RAW_INFO("\n"); 
          
          current.position.x = values[0];
          current.position.y = values[1];
          current.unwrapped_rotation = values[2];
          
          current.velocity.x = values[3];
          current.velocity.y = values[4];

          m1.position.x = values[5];
          m1.position.y = values[6];

          m2.position.x = values[7];
          m2.position.y = values[8];

          uint8_t flags;
          flags = *((uint8_t*) (values+9));

          //                  Current pos, current milestone, next milestone, flags
          nav_position_update(&current,    &m1,               &m2,            flags);
        }
        break;

        case DCS_COMMAND_LOAD_PROFILE:
          uint8_t profile_id = data[1];
          //load_profile(profile_id, data + 2, p_evt_write->len - 2);
          const uint8_t *params = data + 2;
          profile_load(profile_id, params, p_evt_write->len - 2);

        break;

        case DCS_COMMAND_ADD_MILESTONES:{
          uint8_t size = data[1];

          NRF_LOG_RAW_INFO("[dcs] Size: %d\n", size);

          float* values = (float*) (data + 2);
          point milestone;
          
          for(uint8_t i = 0; i < size; i++){
            milestone.x = values[2 * i];
            milestone.y = values[2 * i+1];
            nav_add_milestone(&milestone);
          }
        }
        break;
      
        // Clear all current milestones and replace with new ones
        // Useful when initial milestones are based off of a completely incorrect inital
        // position, e.g. in the event of kidnapping
        case DCS_COMMAND_RESET_MILESTONES:{
          uint8_t size = data[1];

          NRF_LOG_RAW_INFO("[dcs] Size: %d\n", size);

          float* values = (float*) (data + 2);
          point milestone;

          nav_clear_milestones(); // Remove all existing milestones to replace with new ones
          
          for(uint8_t i = 0; i < size; i++){
            milestone.x = values[2 * i];
            milestone.y = values[2 * i+1];
            nav_add_milestone(&milestone);
          }

          nav_clear_action_queue(); // Remove all existing actions
        }
        break;

        case DCS_COMMAND_NAV_BEGIN:
          nav_begin();
        break;

        case DCS_COMMAND_ENTER_BASE:{
          bool from_base = data[1];
          int8_t depth = data[2];
          
          if(from_base){
            if(depth != 0){
              ir_set_mode(IR_MODE_COUNTER);

              if(depth > 0){
                ir_set_target_depth(depth);
                motors_forward(150);
              }else{
                motors_backward(150);
                ir_set_target_depth(-depth);
              }

              ir_enable(true);
              
            }

          }else{
            // Charger enter base routine 
            charger_start_station_entry();
          }
        }
        break;

        case DCS_COMMAND_PING:
          NRF_LOG_RAW_INFO("PING RECEIVED\n");
          eventQueuePush(EVENT_PING);
        break;
        
        case DCS_COMMAND_XCORR_STREAM_ENABLE:
          NRF_LOG_RAW_INFO("XCORR STREAM ENABLED\n");
          bool use_notifications = data[1];
          
          nav_enable_xcorr_stream(use_notifications);
        break;

        case DCS_COMMAND_XCORR_STREAM_DISABLE:
          nav_disable_xcorr_stream();
        break;
      }
  }
}

void ble_dcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context){
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


