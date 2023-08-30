#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"

#include "app_timer.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
//#include "bsp_btn_ble.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_config.h"
#include "event.h"
#include "ble_manager.h"
#include "timers.h"
#include "audio.h"
#include "sensors.h"
#include "ringbuffer.h"
#include "point.h"
#include "nav.h"

#include "nrf_nvic.h"

// Custom services
#include "ble_dcs.h"
#include "ble_ass.h"
#include "ble_sss.h"

#include "led.h"

BLE_CUS_DEF(m_dcs, dcs)
BLE_CUS_DEF(m_ass, ass)
BLE_CUS_DEF(m_sss, sss)
//NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
//               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
//               NRF_BLE_GQ_QUEUE_SIZE);


#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

#define STATE_DUMP_SIZE (sizeof(float) * 5)
// update type + offset sample + 2 states
#define XCORR_DATASIZE (1 + sizeof(int16_t) + 2 * STATE_DUMP_SIZE)
// update type + 1 state
#define MILESTONE_NOTIF_DATASIZE (1 + STATE_DUMP_SIZE)

//extern ble_cus_t m_dcs;
//extern ble_cus_t m_ass;

static volatile bool m_connected;
static uint8_t* transmitData;
static uint16_t transmitOffset;
static uint32_t transmitLength;
static uint16_t maxAttMtuBytes;
static uint8_t bleCusPacket[256] = {0};

#define NUM_SERVICES 3
/**< Universally unique service identifiers. */
static ble_uuid_t m_adv_uuids[] = 
{
  {DEVICE_CONTROL_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN},
  //{AUDIO_STREAMING_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}

};

static ble_cus_t* SERVICES[NUM_SERVICES] = 
{
  &m_dcs,
  &m_ass,
  &m_sss,
};


#define RING_BUFFER_SIZE 2048

#define INDICATION_RB_SIZE 1024
#define NOTIFICATION_RB_SIZE 1024

static ringbuffer_t indication_rb;
static uint8_t _ind_rb[INDICATION_RB_SIZE];

static ringbuffer_t notification_rb;
static uint8_t _notif_rb[NOTIFICATION_RB_SIZE];

static ringbuffer_t mic_rb;
static uint8_t _mic_rb[RING_BUFFER_SIZE];
static uint16_t sequenceNumber = 1;

//static uint8_t ringBuffer[RING_BUFFER_SIZE] = {0};
//static uint16_t ringBufferHead = 0;
//static uint16_t ringBufferTail = 0;
//static int ringBufferBytesUsed = 0;


char const * phy_str(ble_gap_phys_t phys)
{
  static char const * str[] =
  {
    "1 Mbps",
    "2 Mbps",
    "Coded",
    "Unknown"
  };

  switch (phys.tx_phys)
  {
    case BLE_GAP_PHY_1MBPS:
      return str[0];

    case BLE_GAP_PHY_2MBPS:
    case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
    case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
      return str[1];

    case BLE_GAP_PHY_CODED:
      return str[2];

    default:
      return str[3];
  }
}

static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
  switch (p_evt->evt_id)
  {
    case NRF_BLE_GATT_EVT_ATT_MTU_UPDATED:
    {
      maxAttMtuBytes = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
      NRF_LOG_INFO("ATT MTU exchange completed. MTU set to %u bytes.",
                    p_evt->params.att_mtu_effective);
      break;
    }

    case NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED:
    {
      NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);

      uint8_t dataLength;
      nrf_ble_gatt_data_length_get(&m_gatt, m_conn_handle, &dataLength);
      NRF_LOG_INFO("nrf_ble_gatt_data_length_get: %u", dataLength);

      uint16_t effMtu = nrf_ble_gatt_eff_mtu_get(&m_gatt, m_conn_handle);
      NRF_LOG_INFO("nrf_ble_gatt_eff_mtu_get: %u", effMtu);
      break;
    }
  }
  for(int i = 0; i < NUM_SERVICES; i++){
    ble_cus_on_gatt_evt(SERVICES[i], p_evt);  
  }
}

static void gatt_init(void)
{
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
  APP_ERROR_CHECK(err_code);
}

static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void services_init(void)
{
  ret_code_t         err_code;
  nrf_ble_qwr_init_t qwr_init = {0};

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  ble_srv_cccd_security_mode_t custom_value_char_attr_md = {0};
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&custom_value_char_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&custom_value_char_attr_md.write_perm);


  // Initialize device control service  
  err_code = ble_dcs_init(&m_dcs);
  APP_ERROR_CHECK(err_code);

  //m_dcs.p_gatt_queue = &m_ble_gatt_queue;

  // Initialize audio streaming service  
  err_code = ble_ass_init(&m_ass);
  APP_ERROR_CHECK(err_code);

  err_code = ble_sss_init(&m_sss);
  APP_ERROR_CHECK(err_code);

  //m_ass.p_gatt_queue = &m_ble_gatt_queue;
}

static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
  switch(p_evt->evt_type)
  {
    case BLE_CONN_PARAMS_EVT_SUCCEEDED:
      break;

    case BLE_CONN_PARAMS_EVT_FAILED:
      //err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
      //APP_ERROR_CHECK(err_code);
      NRF_LOG_ERROR("BLE_CONN_PARAMS_EVT_FAILED. Keep the connection anyway..");
      break;

    default:
      NRF_LOG_INFO("Unhandled conn params event %d", p_evt->evt_type);
      break;
  }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

static void conn_params_init(void)
{
  ret_code_t             err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = on_conn_params_evt;
  cp_init.error_handler                  = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  ret_code_t err_code;

  switch (ble_adv_evt)
  {
    case BLE_ADV_EVT_FAST:
      NRF_LOG_RAW_INFO("%08d [ble] advertising\n", systemTimeGetMs());
      //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
      err_code = led_indicate(LED_ADVERTISING);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_ADV_EVT_IDLE:
      NRF_LOG_RAW_INFO("%08d [ble] idle\n", systemTimeGetMs());
      eventQueuePush(EVENT_BLE_IDLE);
      break;

    default:
      break;
  }
}

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code = NRF_SUCCESS;

  switch (p_ble_evt->header.evt_id)
  {
    case BLE_GAP_EVT_DISCONNECTED:
      NRF_LOG_INFO("Disconnected.");
      eventQueuePush(EVENT_BLE_DISCONNECTED);
      // LED indication will be changed when advertising starts.
      m_connected = false;
      break;

    case BLE_GAP_EVT_CONNECTED:
      NRF_LOG_INFO("Connected.");
      //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
      err_code = led_indicate(LED_CONNECTED);
      APP_ERROR_CHECK(err_code);
      m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
      err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
      APP_ERROR_CHECK(err_code);
      m_connected = true;

      //err_code = nrf_ble_gq_conn_handle_register(&m_ble_gatt_queue, m_conn_handle);
      //APP_ERROR_CHECK(err_code);
      break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
    {
      uint16_t max_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval;
      uint16_t min_con_int = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval;
      NRF_LOG_INFO("Connection interval updated: %d, %d", (5*min_con_int)/4, (5*max_con_int)/4);
      //ble_cus_send_notification(&m_dcs, &m_dcs.characteristic_handles[DCS_STATUS_CHAR_HANDLE], byte, 1);
      uint8_t byte = 0xFF;
      ble_indicate_status(DCS_STATUS_CONNECTION_INTERVAL_UPDATE, &byte, 1);

      break;
    }

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      ble_gap_phys_t const phys =
      {
          .rx_phys = BLE_GAP_PHY_2MBPS,//BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_2MBPS,//BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
      break;
    }

    case BLE_GAP_EVT_PHY_UPDATE:
    {
      ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

      if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
      {
        // Ignore LL collisions.
        NRF_LOG_DEBUG("LL transaction collision during PHY update.");
        break;
      }

      ble_gap_phys_t phys = {0};
      phys.tx_phys = p_phy_evt->tx_phy;
      phys.rx_phys = p_phy_evt->rx_phy;
      NRF_LOG_INFO("PHY update %s. PHY set to %s.",
                    (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
                    "accepted" : "rejected",
                    phy_str(phys));
      break;
    }

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
      NRF_LOG_INFO("Pairing not supported");
      err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
      NRF_LOG_INFO("No system attributes have been stored.");
      err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTC_EVT_TIMEOUT:
      NRF_LOG_INFO("GATT Client Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_TIMEOUT:
      NRF_LOG_INFO("GATT Server Timeout.");
      err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
      APP_ERROR_CHECK(err_code);
      break;

    case BLE_GATTS_EVT_WRITE:
      //NRF_LOG_INFO("BLE_GATTS_EVT_WRITE");
      break;

    case BLE_GATTS_EVT_HVN_TX_COMPLETE:
      transmitDone = true;
      eventQueuePush(EVENT_BLE_SEND_DATA_DONE);
      NRF_LOG_DEBUG("Handle value notification");
      break;

    case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
      NRF_LOG_INFO("BLE_GATTC_EVT_EXCHANGE_MTU_RSP");
      break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
      NRF_LOG_INFO("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST");
      break;

    case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
      NRF_LOG_INFO("BLE_GAP_EVT_DATA_LENGTH_UPDATE");
      break;

    case BLE_GAP_EVT_ADV_SET_TERMINATED:
      NRF_LOG_INFO("BLE_GAP_EVT_ADV_SET_TERMINATED");
      break;
    
    case BLE_GATTS_EVT_HVC:
      NRF_LOG_INFO("[ble_manager] Handle Value Confirmation");
      ble_cus_indication_confirm();

      //if(ble_cus_indication_in_progress()){
      //  NRF_LOG_INFO("[ble_manager]  ***** INDICATION IN PROGRESS *****");
      //}else{
      //  NRF_LOG_INFO("[ble_manager]  ===== INDICATION FALSE =====");
      //}
      
      // Put all indication requests here.
      ble_indication_buffer_process();
      
      //if(ble_cus_indication_in_progress()){
      //  NRF_LOG_INFO("[HANDLE]  ***** INDICATION IN PROGRESS *****");
      //}else{
      //  NRF_LOG_INFO("[HANDLE]  ===== INDICATION FALSE =====");
      //}

    break;

    default:
      NRF_LOG_INFO("BLE event not handled by app: %i", p_ble_evt->header.evt_id);
      break;
  }
}

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

static void advertising_init(void)
{
  ret_code_t             err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance      = false;
  init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_fast_enabled  = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

void SWI1_IRQHandler(bool radio_evt)
{
  // if (radio_evt) { eventQueuePush(EVENT_BLE_RADIO_START); }
}

uint32_t radio_notification_init(uint32_t irq_priority, uint8_t notification_type, uint8_t notification_distance)
{
  uint32_t err_code;

  err_code = sd_nvic_ClearPendingIRQ(SWI1_IRQn);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  err_code = sd_nvic_SetPriority(SWI1_IRQn, irq_priority);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  err_code = sd_nvic_EnableIRQ(SWI1_IRQn);
  if (err_code != NRF_SUCCESS) {
    return err_code;
  }

  // Configure the event
  return sd_radio_notification_cfg_set(notification_type, notification_distance);
}

// PUBLIC
//#define TX_POWER 4
#define TX_POWER 8
//(accepted values are -40, -20, -16, -12, -8, -4, 0, and 4 dBm)
void bleAdvertisingStart()
{
  ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, TX_POWER);
  err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

void bleInit(void)
{
  ble_stack_init();
  ret_code_t err_code = radio_notification_init(3, NRF_RADIO_NOTIFICATION_TYPE_INT_ON_ACTIVE, NRF_RADIO_NOTIFICATION_DISTANCE_800US);
  APP_ERROR_CHECK(err_code);
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  ble_cus_indication_confirm();
  transmitDone = true;
  m_connected = false;

  uint8_t address[6] = {0};
  address[0] = 0xC0 | ((NRF_FICR->DEVICEADDR[1] & 0xFF00) >> 8);
  address[1] = (NRF_FICR->DEVICEADDR[1] & 0xFF);
  address[2] = (NRF_FICR->DEVICEADDR[0] & 0xFF000000) >> 24;
  address[3] = (NRF_FICR->DEVICEADDR[0] & 0xFF0000) >> 16;
  address[4] = (NRF_FICR->DEVICEADDR[0] & 0xFF00) >> 8;
  address[5] = (NRF_FICR->DEVICEADDR[0] & 0xFF);

  // Contains 2 points and a sample offset
  //rb_create(&xcorr_rb, 20 * (XCORR_DATASIZE) );
  //rb_create(&milestone_rb, 20 * (MILESTONE_NOTIF_DATASIZE));
  //rb_create(&indication_rb, INDICATION_RB_SIZE);
  rb_init(&indication_rb, _ind_rb, INDICATION_RB_SIZE);
  rb_init(&notification_rb, _notif_rb, NOTIFICATION_RB_SIZE);
  rb_init(&mic_rb, _mic_rb, RING_BUFFER_SIZE);

  NRF_LOG_RAW_INFO("%08d [ble] address -> ", systemTimeGetMs());
  NRF_LOG_RAW_INFO("%02X:%02X:%02X:%02X:%02X:%02X\n", address[0], address[1], address[2], address[3], address[4], address[5]);
}


uint16_t getPacketCount(uint16_t length){
  return (length - 1) / PACKET_LENGTH + 1;
}

// Recordings can have occasional dropped packets
bool ble_manager_send_mic_data(uint8_t *packet, uint16_t length, bool use_indication){
  if(!use_indication){
    return ble_cus_send_notification(&m_ass, &m_ass.characteristic_handles[ASS_MIC_CHAR_HANDLE], packet, length);
  }else{
    return ble_cus_send_indication(&m_ass, &m_ass.characteristic_handles[ASS_MIC_CHAR_HANDLE], packet, length);
  }
}

// Metadata is sent via indication because receiver will stall otherwise
bool ble_manager_send_metadata(uint8_t *metadata, uint16_t metadata_length){
  NRF_LOG_RAW_INFO("[ble_manager] sending metadata\n");
  return ble_cus_send_indication(&m_ass, &m_ass.characteristic_handles[ASS_MIC_CHAR_HANDLE], metadata, metadata_length);
}

bool ble_audio_stream_buffer_available(void){
  return rb_size(&mic_rb) > 0;
}

bool ble_status_notification_buffer_available(void){
  return rb_size(&notification_rb) > 0;
}

bool ble_status_indication_buffer_available(void){
  return rb_size(&indication_rb) > 0;
}

void send(void){

  //NRF_LOG_INFO("[ble_manager] send()");
  //int length = maxAttMtuBytes;

  while(transmitDone && ble_audio_stream_buffer_available() ) {
    //NRF_LOG_INFO("[ble_manager] Ok");
    
    //int length = ringBuffer[ringBufferHead];
    //NRF_LOG_RAW_INFO("Transmitting length: %d\n", length);
    int length = rb_at(&mic_rb, 0);

    //for (int i = 1; i < length; i++){
    //  bleCusPacket[i-1] = ringBuffer[(ringBufferHead + i) % RING_BUFFER_SIZE];
    //}

    rb_get_fast_offset(&mic_rb, bleCusPacket, length - 1, 1);

    transmitDone = ble_manager_send_mic_data(bleCusPacket, length-1, 0);
    //transmitDone = ble_manager_send_mic_data(bleCusPacket, length-1, audio_compression_enabled());

    if (transmitDone) {
      //ringBufferHead = (ringBufferHead + length) % RING_BUFFER_SIZE;
      //ringBufferBytesUsed -= length;
      rb_advance(&mic_rb, length);
      //NRF_LOG_RAW_INFO("RB head%d\n", mic_rb.head);
    }else{
      //NRF_LOG_RAW_INFO("Failed to transmit\n");
      //NRF_LOG_RAW_INFO("Length %d, ringBufferBytesUsed %d\n", length - 1, ringBufferBytesUsed);
      //NRF_LOG_RAW_INFO("Failed to transmit\n");
    }
  }

  //sending = false;
}

void bleSendData(uint8_t *data, int length)
{
  //uint16_t n_packets = getPacketCount(length);
  
  int i = 0;

  //// TODO: Optimize  
  //ringBuffer[(ringBufferTail) % RING_BUFFER_SIZE] = length + 3;
  //ringBuffer[(ringBufferTail + 1) % RING_BUFFER_SIZE] = (uint8_t) (sequenceNumber >> 8) & 0xFF;
  //ringBuffer[(ringBufferTail + 2) % RING_BUFFER_SIZE] = (uint8_t) (sequenceNumber) & 0xFF;
  //for (int j = 0; j < length; i++, j++) {
  //  //ringBuffer[(ringBufferTail + i) % RING_BUFFER_SIZE] = data[i-2];
  //  if(i < length) ringBuffer[(ringBufferTail + j + 3) % RING_BUFFER_SIZE] = data[i];
  //  else ringBuffer[(ringBufferTail + j + 3) % RING_BUFFER_SIZE] = 0;
  //}

  //ringBufferTail = (ringBufferTail + (length+3) ) % RING_BUFFER_SIZE;
  //ringBufferBytesUsed += (length + 3);
  uint8_t byte = length + 3;
  rb_put(&mic_rb, &byte, 1);
  byte = (uint8_t) (sequenceNumber >> 8) & 0xFF;
  rb_put(&mic_rb, &byte, 1);
  byte = (uint8_t) (sequenceNumber) & 0xFF;
  rb_put(&mic_rb, &byte, 1);

  rb_put(&mic_rb, data, length);

  sequenceNumber++;

  //NRF_LOG_RAW_INFO("transmitDone: %d, Ring buffer bytes used: %d/%d\n", transmitDone, rb_size(&mic_rb), RING_BUFFER_SIZE);

  send();
}

bool bleBufferHasSpace(uint8_t length){
  return length + sizeof(sequenceNumber) + 1 + rb_size(&mic_rb) < mic_rb.size - 1;
}

void blePushSequenceNumber(void)
{
  // (sizeof(int16_t) * PDM_DECIMATION_BUFFER_LENGTH)/maxAttMtuBytes
  //sequenceNumber += PDM_PACKETS_PER_TRANSMISSION;
  sequenceNumber += 1;//getPacketCount(audio_buffer_length());
}

uint16_t getSequenceNumber(void){
  return sequenceNumber;
}

static uint16_t xcorr_sequence_number = 0;

void resetBLE(void){
  if(sequenceNumber > 0) sequenceNumber = 1;
  //ringBufferHead = ringBufferTail = 0;
  //ringBufferBytesUsed = 0;
  
  rb_reset(&mic_rb);
  NRF_LOG_RAW_INFO("[ble_manager] RESET BLE\n");

  xcorr_sequence_number = 0;
  transmitDone = true;
}

void ble_reset_tx_state(void){
  transmitDone = true;
}

void ble_reset_sequence_number(void){
  sequenceNumber = 1;
}


void ble_notify_recording_finished(void){
uint8_t byte = ASS_STATUS_RECORDING_FINISHED;
ble_cus_send_notification(&m_ass, &m_ass.characteristic_handles[ASS_STATUS_CHAR_HANDLE], &byte, 1);
}

void bleSendSensorData(sensor_t *sensor){
  ble_sss_notify_sensor_data(&m_sss, &m_sss.characteristic_handles[SSS_SENSOR_DATA_HANDLE], sensor);
}

uint8_t ble_dump_nav_state(float *data, nav_state_t *state){
  data[0] = state->position.x;
  data[1] = state->position.y;
  
  data[2] = state->unwrapped_rotation;

  data[3] = state->velocity.x;
  data[4] = state->velocity.y;

  return 5 * sizeof(float);
}

void ble_dump_xcorr_offset(uint8_t *data, uint16_t xcorr_delay){
  data[0] = MSB_16(xcorr_sequence_number);
  data[1] = LSB_16(xcorr_sequence_number);
  xcorr_sequence_number++;

  data[2] = MSB_16(xcorr_delay);
  data[3] = LSB_16(xcorr_delay);
}

void ble_indication_buffer_process(void){
  if(!rb_empty(&indication_rb)){
    //NRF_LOG_RAW_INFO("RB Size: %d\n", rb_size(&indication_rb));

    static uint8_t data[256];
    
    uint8_t length = rb_at(&indication_rb, 0);
    rb_get_fast_offset(&indication_rb, data, length, 1);

    if(ble_cus_send_indication(&m_dcs, &m_dcs.characteristic_handles[DCS_STATUS_CHAR_HANDLE], data, length)){
      rb_advance(&indication_rb, 1 + length);
      //NRF_LOG_RAW_INFO("[ble_manager] SUCCESS: %d\n", length);
    }
  }
}

void ble_notification_buffer_process(void){
  if(!rb_empty(&notification_rb)){
    //NRF_LOG_RAW_INFO("RB Size: %d\n", rb_size(&notification_rb));

    static uint8_t data[256];
    
    uint8_t length = rb_at(&notification_rb, 0);
    rb_get_fast_offset(&notification_rb, data, length, 1);

    if(ble_cus_send_notification(&m_dcs, &m_dcs.characteristic_handles[DCS_STATUS_CHAR_HANDLE], data, length)){
      rb_advance(&notification_rb, 1 + length);
      //NRF_LOG_RAW_INFO("[ble_manager] SUCCESS: %d\n", length);
    }
  }
}

void ble_notify_status(uint8_t status_code, uint8_t *status, uint8_t length){
  if(rb_size(&notification_rb) + (length + 1) >= notification_rb.size - 1) {
    return;
  }
  
  length++;
  rb_put(&notification_rb, &length, 1);
  rb_put(&notification_rb, &status_code, 1);
  rb_put(&notification_rb, status, length - 1);

  ble_notification_buffer_process();
}

void ble_indicate_status(uint8_t status_code, uint8_t *status, uint8_t length){
  // TODO: Check for space?
  if(rb_size(&indication_rb) + (length + 1) >= indication_rb.size - 1) {
    //NRF_LOG_RAW_INFO("%08d [ble_manager] Indication buffer full, dropping packet (%d / %d)\n", systemTimeGetUs(), rb_size(&indication_rb), INDICATION_RB_SIZE);
    return;
  }
  
  if(!m_connected){
    NRF_LOG_RAW_INFO("[ble_manager] ble_indicate_status: No device connected, not sending\n");
    return;
  }else{
    //NRF_LOG_RAW_INFO("[ble_manager] SENDING STATUS UPDATE %d LENGTH %d\n", status_code, length);
  }

  length++; // Include status code byte

  CRITICAL_REGION_ENTER();
  rb_put(&indication_rb, &length, 1);
  rb_put(&indication_rb, &status_code, 1);
  rb_put(&indication_rb, status, length - 1);
  CRITICAL_REGION_EXIT();

  ble_indication_buffer_process();
}

void ble_add_xcorr_data(uint16_t xcorr_delay, nav_state_t *s1, nav_state_t *s2){
  static uint8_t data[256];
  
  ble_dump_xcorr_offset(data, xcorr_delay);
  
  float* floats = (float*) (data + 4);
  ble_dump_nav_state(floats, s1);
  ble_dump_nav_state(floats + STATE_DUMP_SIZE / sizeof(float), s2);

  uint8_t length = 4 + 2 * STATE_DUMP_SIZE;
  ble_indicate_status(DCS_STATUS_UPDATE_XCORR, data, length);
}

void ble_indicate_milestone_reached(nav_state_t *current_state){
  static uint8_t data[256];

  ble_dump_nav_state((float*) data, current_state);

  uint8_t length = STATE_DUMP_SIZE;
  ble_indicate_status(DCS_STATUS_MILESTONE_REACHED, data, length);
}

void ble_indicate_motion_correction(nav_state_t *current_state){
  static uint8_t data[256];

  ble_dump_nav_state((float*) data, current_state);

  uint8_t length = STATE_DUMP_SIZE;
  ble_indicate_status(DCS_STATUS_MOTION_CORRECTION, data, length);
}

void ble_indicate_next_milestone(nav_state_t *current_state){
  static uint8_t data[256];

  ble_dump_nav_state((float*) data, current_state);

  uint8_t length = STATE_DUMP_SIZE;
  ble_indicate_status(DCS_STATUS_NEXT_MILESTONE, data, length);
}

void ble_indicate_transition(uint8_t before, uint8_t after){
  static uint8_t data[256];
  data[0] = before;
  data[1] = after;
  uint8_t length = 2;
  //NRF_LOG_INFO("[ble_manager] Status Length: %d", length);
  ble_indicate_status(DCS_STATUS_NAV_TRANSITION, data, length);
}

void ble_reset_xcorr_sequence_number(void){
  xcorr_sequence_number = 0;
}