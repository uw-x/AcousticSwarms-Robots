#include "sdk_common.h"
#include "ble_cus.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "event.h"
#include "nrf_log.h"
#include "peripheral.h"

#include "ble_db_discovery.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "nrf_sdh.h"
#include "sdk_config.h"

#include <stdlib.h>
#include "usb.h"
#include "ringbuffer.h"
#include "timers.h"
#include "event.h"

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO   3                                       /**< BLE observer priority of the application. There is no need to modify this value. */

#define PERIPHERAL_NAME "shio"

#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#define SCAN_DURATION 5000 // 5 second scan

BLE_CUS_DEF(m_cus);
NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

#define WRITE_BUFFER_SIZE 16384

static uint8_t write_buffer_arr[WRITE_BUFFER_SIZE];
static ringbuffer_t write_buffer;
static uint8_t m_packet[256];
static bool ble_transmit_done = true;
uint32_t scan_start_time;
static bool m_scan_stopped = false;
static uint16_t m_ble_cus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


static ble_uuid_t const m_uuids[] = {
      {DEVICE_CONTROL_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN},
      {AUDIO_STREAMING_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN},
      {SENSOR_STREAMING_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN},
};

bool ble_write_buffer_empty(){
  return rb_size(&write_buffer) == 0;
}

static uint8_t get_device_by_handle(uint16_t conn_handle){
  uint8_t idx = 0;
  for(; idx < NRF_SDH_BLE_CENTRAL_LINK_COUNT; idx++){
    if(m_cus.ble_device_list[idx] != NULL &&
       m_cus.ble_device_list[idx]->conn_handle == conn_handle) break;
  }
  if(idx == NRF_SDH_BLE_CENTRAL_LINK_COUNT){
    NRF_LOG_RAW_INFO("Could not find device %d from handle!", conn_handle);
  }
  return idx;
}

static bool get_characteristic_by_uuid(ble_device_t *dev, uint16_t uuid, ble_cus_characteristic_t *ch){
  uint8_t s, c;
  for(s=0; s < dev->n_services; s++){
    //NRF_LOG_INFO("s: %d\t\n", s);
    for(c=0; c < dev->services[s].n_characteristics; c++){
      if(uuid == dev->services[s].characteristics[c].uuid){
        *ch = dev->services[s].characteristics[c];
        return true;
      }
    }
  }
  return false;
}

static bool get_characteristic_by_handle(ble_device_t *dev, uint16_t handle, ble_cus_characteristic_t *ch){
  uint8_t s, c;
  for(s=0; s < dev->n_services; s++){
    for(c=0; c < dev->services[s].n_characteristics; c++){
      if(handle == dev->services[s].characteristics[c].value_handle){
        *ch = dev->services[s].characteristics[c];
        return true;
      }
    }
  }
  return false;
}

static void gatt_error_handler(uint32_t nrf_error, void * p_ctx, uint16_t conn_handle){
    NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);
}


uint32_t ble_cus_attempt_transmit(bool bypass){
    //NRF_LOG_RAW_INFO("ble_cus_attempt_transmit\n");
    //if(!bypass && !ble_transmit_done){
    //  //NRF_LOG_RAW_INFO("Busy\n");
    //  return !NRF_SUCCESS;
    //}
    
    uint16_t conn_handle, value_handle, length;
    uint8_t data[6];
    rb_get_fast(&write_buffer, data, 6);
    conn_handle = data[0] + (data[1] << 8);
    value_handle = data[2] + (data[3] << 8);
    length = data[4] + (data[5] << 8);
    rb_get_fast_offset(&write_buffer, m_packet, length, 6);

    //conn_handle = rb_at(&write_buffer, 0) + (rb_at(&write_buffer, 1) << 8);
    //value_handle = rb_at(&write_buffer, 2) + (rb_at(&write_buffer, 3) << 8);
    //length = rb_at(&write_buffer, 4) + (rb_at(&write_buffer, 5) << 8);
    
    //for(uint16_t i = 0; i < length; i++){
    //  m_packet[i] = rb_at(&write_buffer, 6 + i);
    //}
    
    nrf_ble_gq_req_t write_req;
    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    write_req.error_handler.cb            = gatt_error_handler;
    write_req.params.gattc_write.handle   = value_handle;
    write_req.params.gattc_write.len      = length;
    write_req.params.gattc_write.offset   = 0;
    write_req.params.gattc_write.p_value  = m_packet;
    write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    ble_transmit_done = false;
    ret_code_t ret = nrf_ble_gq_item_add(m_cus.p_gatt_queue, &write_req, conn_handle);
    if(ret == NRF_SUCCESS){
      rb_advance(&write_buffer, 6 + length);
      NRF_LOG_RAW_INFO("Success!\n");
    }else{
      NRF_LOG_RAW_INFO("Writing failed, kept in queue: 0x%x\n", ret);
    }

    return ret;
}

/**@brief Function for creating a message for writing to the CCCD. */
uint32_t cccd_configure(uint16_t conn_handle, uint16_t uuid, bool notification_enable, bool bypass){
    uint16_t length, cccd_handle, cccd_val;
    if(sizeof(conn_handle) + sizeof(length) + sizeof(cccd_handle) + sizeof(cccd_val) > WRITE_BUFFER_SIZE - 1){
      NRF_LOG_INFO("WARNING: No space in BLE queue, dropping BLE request!\n");
      return false;
    }

    uint8_t i = get_device_by_handle(conn_handle);
    ble_cus_characteristic_t ch;
    if(get_characteristic_by_uuid(m_cus.ble_device_list[i], uuid, &ch)){
      cccd_handle = ch.cccd_handle;
      NRF_LOG_RAW_INFO("Found characteristic with uuid %d, cccd handle is: %d.\n", uuid, cccd_handle);
    }else{
      NRF_LOG_INFO("Could not find uuid %d.", uuid);
    }
    
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    cccd_val = notification_enable ? (BLE_GATT_HVX_NOTIFICATION | BLE_GATT_HVX_INDICATION) : 0;
    length = BLE_CCCD_VALUE_LEN;
    
    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    rb_put(&write_buffer, (uint8_t *) &conn_handle, sizeof(conn_handle));
    rb_put(&write_buffer, (uint8_t *) &cccd_handle, sizeof(cccd_handle));
    rb_put(&write_buffer, (uint8_t *) &length, sizeof(length));
    rb_put(&write_buffer, (uint8_t *) cccd, length);

    return ble_cus_attempt_transmit(bypass) == NRF_SUCCESS;
    //return nrf_ble_gq_item_add(m_cus.p_gatt_queue, &cccd_req, m_cus.ble_device_list[i]->conn_handle);
    //return 0;
}

// Each mic data packet is 242 bytes long. Receiver adds:
// - 1 byte for metadata
// - 2 bytes for robot id
// - 2 bytes for packet length + robot id length (2 + 242 == 244)
// - In total, the USB packet should be 247 bytes long
void ble_cus_write_mic_data(uint16_t conn_handle, const uint8_t* const data, uint16_t length){
  uint8_t conn_handle_arr[2] = {MSB_16(conn_handle), LSB_16(conn_handle)};
  //usb_write2(USB_MIC_DATA, sizeof(conn_handle), conn_handle_arr, length, data);
  uint16_t sequence_number = (data[0] << 8) + data[1];
  if(usb_write2(USB_MIC_DATA, sizeof(conn_handle), conn_handle_arr, length, data)){
    //NRF_LOG_RAW_INFO("Packet succesfully relayed\n");
  }else{
    //for(int i = 0; i < length; i++){
    //  NRF_LOG_RAW_INFO("%d ", data[i]);
    //}NRF_LOG_RAW_INFO("\n");
    //NRF_LOG_RAW_INFO("Packet relay failed\n");
  }
}

void ble_cus_write_sensor_data(uint16_t conn_handle, const uint8_t* const data, uint16_t length){
  uint8_t conn_handle_arr[2] = {MSB_16(conn_handle), LSB_16(conn_handle)};
  uint16_t sequence_number = (data[0] << 8) + data[1];
  if(usb_write2(USB_SENSOR_DATA, sizeof(conn_handle), conn_handle_arr, length, data)){
    //NRF_LOG_RAW_INFO("Packet succesfully relayed\n");
  }else{
    //for(int i = 0; i < length; i++){
    //  NRF_LOG_RAW_INFO("%d ", data[i]);
    //}NRF_LOG_RAW_INFO("\n");
    //NRF_LOG_RAW_INFO("Packet relay failed\n");
  }
}

void ble_cus_handle_notification(const ble_gattc_evt_t* const evt){
  uint16_t conn_handle = evt->conn_handle;
  uint16_t char_handle = evt->params.hvx.handle;
  uint16_t data_len = evt->params.hvx.len;
  const uint8_t *data = evt->params.hvx.data;
  
  uint8_t dev_id = get_device_by_handle(conn_handle);
  
  ble_cus_characteristic_t ch;
  if(!get_characteristic_by_handle(m_cus.ble_device_list[dev_id], char_handle, &ch)){
    NRF_LOG_INFO("Could not find characteristic with handle %d\n", char_handle);
    return;
  }

  switch(ch.uuid){
    case DCS_STATUS_CHAR_UUID:{
      NRF_LOG_INFO("Received status notification\n", char_handle);
      uint8_t conn_handle_arr[2] = {MSB_16(conn_handle), LSB_16(conn_handle)};
      usb_write2(USB_BLE_STATUS_UPDATE, sizeof(conn_handle), conn_handle_arr, data_len, data);
      //if(data_len > 0 && data[0] == 0xFF){
      //  NRF_LOG_INFO("Robot %d has updated connection interval! Informing host\n", conn_handle);
      
      //  uint8_t conn_handle_arr[2] = {MSB_16(conn_handle), LSB_16(conn_handle)};
      //  if(usb_write(USB_BLE_DEVICE_CONNECTED, conn_handle_arr, sizeof(conn_handle))){
      //    NRF_LOG_INFO("Success\n", conn_handle);
      //  }else{
      //    NRF_LOG_INFO("Fail\n", conn_handle);
      //  }
      //}
    }
    break;
    case ASS_MIC_CHAR_UUID:
    ble_cus_write_mic_data(conn_handle, data, data_len);
    break;

    case ASS_STATUS_CHAR_UUID:
    NRF_LOG_INFO("Received recording status notification: %d \n", char_handle, data[0]);
    //if(data_len > 0 && data[0] == 0x01){      
      uint8_t conn_handle_arr[2] = {MSB_16(conn_handle), LSB_16(conn_handle)};
      usb_write(USB_BLE_RECORDING_FINISHED, conn_handle_arr, sizeof(conn_handle));
    //}
    break;
    
    case SSS_SENSOR_DATA_UUID:
    //NRF_LOG_INFO("Received sensor data: %d \n", char_handle, data[0]);
    ble_cus_write_sensor_data(conn_handle, data, data_len);
    break;

    default:
    NRF_LOG_INFO("Unhandled Notification\n");
    break;
  }
}

bool ble_cus_write_char(uint16_t conn_handle, uint16_t uuid, uint8_t *data, uint16_t length){
    if(sizeof(conn_handle) + sizeof(length) + sizeof(uint16_t) + length > WRITE_BUFFER_SIZE - 1){
      NRF_LOG_INFO("WARNING: No space in BLE queue, dropping BLE request!\n");
      return false;
    }
    
    uint8_t i = get_device_by_handle(conn_handle);
    ble_cus_characteristic_t ch;
    uint16_t value_handle;
    
    if(get_characteristic_by_uuid(m_cus.ble_device_list[i], uuid, &ch)){
      value_handle = ch.value_handle;
      //NRF_LOG_RAW_INFO("Found characteristic with uuid %d, value handle is: %d.\n", uuid, value_handle);
    }else{
      NRF_LOG_INFO("Could not find uuid %d.", uuid);
    }

    rb_put(&write_buffer, (uint8_t *) &conn_handle, sizeof(conn_handle));
    rb_put(&write_buffer, (uint8_t *) &value_handle, sizeof(value_handle));
    rb_put(&write_buffer, (uint8_t *) &length, sizeof(length));
    rb_put(&write_buffer, (uint8_t *) data, length);

    return ble_cus_attempt_transmit(false) == NRF_SUCCESS;
}

void ble_cus_read_char(uint16_t char_uuid, uint8_t *data, uint16_t length){

}

uint32_t ble_cus_init(nrf_ble_gq_t *gq){
  m_cus.p_gatt_queue = gq;
  m_cus.device_count = 0;

  uint32_t      err_code;
  ble_uuid_t    uuid;
  ble_uuid128_t base_uuid = {DEVICE_CONTROL_SERVICE_UUID_BASE};

  // Register UUID
  int n_services = sizeof(m_uuids) / sizeof(m_uuids[0]);
  for(int i = 0; i < n_services; i++){
    err_code = sd_ble_uuid_vs_add(&base_uuid, &m_cus.uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid_t    service_uuid;
    service_uuid.uuid = m_uuids[i].uuid;
    service_uuid.type = m_cus.uuid_type;

    err_code = ble_db_discovery_evt_register(&service_uuid);
    VERIFY_SUCCESS(err_code);
  }

  rb_init(&write_buffer, write_buffer_arr, WRITE_BUFFER_SIZE);

  return err_code;
}

uint32_t ble_cus_add_device(ble_gap_addr_t dev_addr, uint16_t conn_handle){
  uint32_t err_code;
  
  uint8_t i = 0;
  for(; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++){
    if(m_cus.ble_device_list[i] == NULL) break;
  }

  m_cus.ble_device_list[i] = (ble_device_t *) malloc(sizeof(ble_device_t));
  m_cus.ble_device_list[i]->addr = dev_addr;
  m_cus.ble_device_list[i]->conn_handle = conn_handle;
  m_cus.ble_device_list[i]->services = calloc(sizeof(m_uuids) / sizeof(m_uuids[0]), sizeof(ble_cus_service_t));
  m_cus.ble_device_list[i]->n_services = 0;

  return nrf_ble_gq_conn_handle_register(m_cus.p_gatt_queue, conn_handle);
}

uint32_t ble_cus_remove_device(uint16_t conn_handle, uint8_t reason){
  uint8_t i = get_device_by_handle(conn_handle);
  //if(i == NRF_SDH_BLE_CENTRAL_LINK_COUNT) return NRF_SUCCESS; // Must've been deleted before.
  
  //NRF_LOG_INFO("Conn handle: %d, i: %d\n", conn_handle, i);
  
  free(m_cus.ble_device_list[i]->services);
  free(m_cus.ble_device_list[i]);
  m_cus.ble_device_list[i] = NULL;

  if(reason != 0x3e){
    uint8_t conn_handle_arr[2] = {MSB_16(conn_handle), LSB_16(conn_handle)};
    usb_write(USB_BLE_DEVICE_DISCONNECTED, conn_handle_arr, sizeof(conn_handle_arr));
    m_cus.device_count--;
  }else{
    event_t ev = {EVENT_DATABASE_DISCOVERY_COMPLETE};
    eventQueuePush(ev);
  }

  return NRF_SUCCESS;
}

void ble_cus_on_service_discovery_done(ble_device_t *dev){
  uint16_t nsrv = sizeof(m_uuids) / sizeof(m_uuids[0]);
  if(dev->n_services == nsrv){
    // Enable notifications for connection interval change
    cccd_configure(dev->conn_handle, DCS_STATUS_CHAR_UUID, true, true);
    m_cus.device_count++;
    
    uint8_t conn_handle_arr[2] = {MSB_16(dev->conn_handle), LSB_16(dev->conn_handle)};
    usb_write(USB_BLE_DATABASE_DISCOVERY_COMPLETE, conn_handle_arr, sizeof(conn_handle_arr));    
    NRF_LOG_INFO("Informing host of connection %d.", dev->conn_handle);
    const uint8_t *address = dev->addr.addr;
    usb_write2(USB_BLE_DEVICE_CONNECTED, sizeof(dev->conn_handle), conn_handle_arr, 6, address);
    
    event_t ev = {EVENT_DATABASE_DISCOVERY_COMPLETE};
    eventQueuePush(ev);

    //// Test
    //uint8_t i = get_device_by_handle(p_cus, conn_handle);
    //ble_device_t *dev = p_cus->ble_device_list[i];

    //ble_cus_characteristic_t ch;
    //NRF_LOG_INFO("%d",get_characteristic_by_uuid(dev, DCS_STATUS_CHAR_UUID, &ch));
    //NRF_LOG_INFO("%d",get_characteristic_by_uuid(dev, DCS_COMMAND_CHAR_UUID, &ch));

    //NRF_LOG_INFO("%d",get_characteristic_by_uuid(dev, ASS_MIC_CHAR_UUID, &ch));
    //NRF_LOG_INFO("%d",get_characteristic_by_uuid(dev, ASS_SPK_CHAR_UUID, &ch));
    //NRF_LOG_INFO("%d",get_characteristic_by_uuid(dev, ASS_CONTROL_CHAR_UUID, &ch));
  }
}

void ble_cus_db_disc_evt(ble_db_discovery_evt_t * p_evt){
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE){
        uint8_t idx = get_device_by_handle(p_evt->conn_handle);
        uint8_t srv_idx = m_cus.ble_device_list[idx]->n_services;
        //NRF_LOG_INFO("Device id: %d\n", idx);
        uint16_t srv_uuid = p_evt->params.discovered_db.srv_uuid.uuid;
        //NRF_LOG_INFO("Found %d characteristics from service %d.", p_evt->params.discovered_db.char_count, srv_uuid);
        ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;
        m_cus.ble_device_list[idx]->services[srv_idx].uuid = srv_uuid;
        m_cus.ble_device_list[idx]->services[srv_idx].n_characteristics = p_evt->params.discovered_db.char_count;
        m_cus.ble_device_list[idx]->n_services++;
        //NRF_LOG_INFO("n_services: %d.", p_cus->ble_device_list[idx]->n_services);
        
        ble_cus_characteristic_t *device_characteristics = calloc(p_evt->params.discovered_db.char_count, sizeof(ble_cus_characteristic_t));
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++){
            device_characteristics[i].uuid = p_chars[i].characteristic.uuid.uuid;
            device_characteristics[i].props = p_chars[i].characteristic.char_props;
            device_characteristics[i].value_handle = p_chars[i].characteristic.handle_value;
            device_characteristics[i].cccd_handle = p_chars[i].cccd_handle;
        }
        m_cus.ble_device_list[idx]->services[srv_idx].characteristics = device_characteristics;
        
        ble_cus_on_service_discovery_done(m_cus.ble_device_list[idx]);
    }
}


uint8_t ble_get_device_count(void){
  return m_cus.device_count;
}

bool ble_cus_scan_stopped(void){
  return m_scan_stopped;
}

ble_cus_t *ble_cus_get_service(void){
  return &m_cus;
}

void ble_cus_write_tx_finished(void){
  uint8_t data = BLE_TX_FINISHED;
  usb_write(USB_BLE_INFO, &data, 1);
}

// =================================== MOVED FROM MAIN.C ===============================//

/**@brief Function to start scanning. */
void scan_start(void){
    ret_code_t ret;
    scan_start_time = systemTimeGetMs();

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);

    m_scan_stopped = false;
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
         case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
         {
              err_code = p_scan_evt->params.connecting_err.err_code;
              APP_ERROR_CHECK(err_code);
         } break;

         case NRF_BLE_SCAN_EVT_CONNECTED:
         {
             ble_gap_evt_connected_t const * p_connected = p_scan_evt->params.connected.p_connected;
    
             // Scan is automatically stopped by the connection.
             NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                      p_connected->peer_addr.addr[0],
                      p_connected->peer_addr.addr[1],
                      p_connected->peer_addr.addr[2],
                      p_connected->peer_addr.addr[3],
                      p_connected->peer_addr.addr[4],
                      p_connected->peer_addr.addr[5]);

         } break;

         case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
         {
             NRF_LOG_INFO("Scan timed out.");
             //scan_start();
         } break;

         default:
             break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;
    static ble_gap_scan_params_t scan_params;

    memset(&init_scan, 0, sizeof(init_scan));
    memset(&scan_params, 0, sizeof(scan_params));

    scan_params.timeout = 500; // In 10 ms units. (5 seconds)
    scan_params.scan_phys     = BLE_GAP_PHY_2MBPS;

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_params_set(&m_scan, &scan_params);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, PERIPHERAL_NAME);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, true);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function to start scanning. */
void scan_stop(void)
{
    ret_code_t ret;
    
    nrf_ble_scan_stop();

    if(ble_get_device_count()){
       ret = bsp_indication_set(BSP_INDICATE_CONNECTED);
    }else{
       ret = bsp_indication_set(BSP_INDICATE_IDLE);
    }
    APP_ERROR_CHECK(ret);

    m_scan_stopped = true;
}


/**@brief Function for handling the Nordic UART Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void cus_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //ble_nus_c_on_db_disc_evt(&m_cus, p_evt);
    //NRF_LOG_INFO("Handling service discovery.");
    ble_cus_db_disc_evt(p_evt);
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");

            // Assigns connection handler 
            err_code = ble_cus_add_device(p_ble_evt->evt.gap_evt.params.connected.peer_addr,
                                          p_gap_evt->conn_handle);
            //err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Added device.");
            
            // FIXME: Maybe switch the tx to coded PHY?
            NRF_LOG_INFO("Requesting switch to 2 Mbps PHY.");
            ble_gap_phys_t const phys = {.rx_phys = BLE_GAP_PHY_2MBPS,
                                        .tx_phys = BLE_GAP_PHY_2MBPS};
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Done.");

            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Starting services discovery.");
            // start discovery of services.
            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_INFO("Done.");          
            
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            ble_cus_remove_device(p_gap_evt->conn_handle, p_gap_evt->params.disconnected.reason);

            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            if(p_gap_evt->params.disconnected.reason == 0x3E){
              event_t e = {EVENT_DATABASE_DISCOVERY_COMPLETE};
              eventQueuePush(e);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_HVX:
        {
            // If server sends an indication, then send a Handle Value Confirmation to the GATT Server.
            if (p_ble_evt->evt.gattc_evt.params.hvx.type == BLE_GATT_HVX_INDICATION)
            {
                NRF_LOG_INFO("[ble_cus] Received INDICATION!");
                err_code = sd_ble_gattc_hv_confirm(p_ble_evt->evt.gattc_evt.conn_handle,
                                                   p_ble_evt->evt.gattc_evt.params.hvx.handle);
                APP_ERROR_CHECK(err_code);
            }

            uint8_t data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;
            ble_cus_handle_notification(&p_ble_evt->evt.gattc_evt);
             //NRF_LOG_RAW_INFO("%s data: ", (p_ble_evt->evt.gattc_evt.params.hvx.type != BLE_GATT_HVX_NOTIFICATION) ? "i" : "n");

            // NRF_LOG_RAW_INFO("data ")
            // NRF_LOG_RAW_HEXDUMP_INFO(p_ble_evt->evt.gattc_evt.params.hvx.data, data_len);
            //usbSendData((uint8_t*) p_ble_evt->evt.gattc_evt.params.hvx.data, data_len);
            // NRF_LOG_RAW_INFO("\r\n");
        }
        break;

        case BLE_GATTC_EVT_WRITE_RSP:
          ble_transmit_done = true;
          event_t ev;
          ev.type = EVENT_BLE_SEND_DATA_DONE;
          eventQueuePush(ev);
        break;

        default:
        break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_cus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_cus_max_data_len, m_ble_cus_max_data_len);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

void ble_init(){
  db_discovery_init();
  ble_stack_init();
  gatt_init();
  ble_cus_init(&m_ble_gatt_queue);
  scan_init();
}
