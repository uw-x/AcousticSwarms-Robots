#include "usb.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "nrf_log.h"
#include "bsp_btn_ble.h"
#include "event.h"
#include "timers.h"
#include "nrf_delay.h"

#include "ringbuffer.h"

#define READ_SIZE 4096 // Max read length
#define USB_PACKET_SIZE 1443 + 5 // (dev id | sequence length | 2 * (3 * 120 samples))

static uint8_t m_metadata_rx_buffer[METADATA_SIZE];
static uint8_t m_rx_buffer[READ_SIZE];
static uint8_t m_tx_buffer[USB_PACKET_SIZE+1];
static uint8_t m_tx_buffer2[USB_PACKET_SIZE+1];

static bool m_send_flag = 0;
static bool m_metadata_read = false;
static volatile bool transmitting = false;
static volatile bool transmitDone = true;

#define USB_WRITE_BUFFSIZE 80000
#define USB_READ_BUFFSIZE 4096

static uint8_t usb_rb_tx_buffer[USB_WRITE_BUFFSIZE];
static uint8_t usb_rb_rx_buffer[USB_READ_BUFFSIZE];


#define min(a, b) (a < b ? a : b)

static ringbuffer_t write_buffer;
static ringbuffer_t read_buffer;

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

uint32_t usb_attempt_transmit();

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            NRF_LOG_INFO("Connected.\n");
            bsp_board_led_on(LED_CDC_ACM_OPEN);

            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                                   m_rx_buffer,
                                                   METADATA_SIZE);
            UNUSED_VARIABLE(ret);
            
            event_t ev = {EVENT_USB_CONNECTED};
            eventQueuePush(ev);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            NRF_LOG_INFO("Disconnected.\n");
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            event_t ev = {EVENT_USB_DISCONNECTED};
            eventQueuePush(ev);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            //bsp_board_led_invert(LED_CDC_ACM_TX);
            //NRF_LOG_INFO("TX DONE.\n");
            transmitDone = true;
            if(!rb_empty(&write_buffer) && !transmitting){
              usb_attempt_transmit();
            }
            //transmitting = false;
            break;
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
            ret_code_t ret;
            do{
              if(m_metadata_read){
                // RX Buffer is filled with data info.
                //NRF_LOG_INFO("Now reading data");
                size_t bytes_read = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
                //NRF_LOG_INFO("Read %d bytes", bytes_read);
                rb_put(&read_buffer, m_rx_buffer, bytes_read);

                event_t ev = {EVENT_USB_DATA_READ};
                eventQueuePush(ev);
                
                // Initiate new metadata read
                m_metadata_read = false;
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, METADATA_SIZE);
              }else{
                // RX Buffer is filled with metadata info.
                //NRF_LOG_INFO("Now reading metadata");
                rb_put(&read_buffer, m_rx_buffer, METADATA_SIZE);
                uint16_t length = (m_rx_buffer[1] << 8) | m_rx_buffer[2];

                m_metadata_read = true;
                ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, length);
              }
            }while(ret == NRF_SUCCESS);
              
              // Setup next read.
              
              UNUSED_VARIABLE(ret);

            break;
        }
        default:
          transmitDone = true;
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;
        case APP_USBD_EVT_POWER_DETECTED:
            NRF_LOG_INFO("USB power detected");

            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
            NRF_LOG_INFO("USB power removed");
            app_usbd_stop();
            break;
        case APP_USBD_EVT_POWER_READY:
            NRF_LOG_INFO("USB ready");
            app_usbd_start();
            break;
        default:
            break;
    }
}

ret_code_t usb_init(){
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler
    };

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD CDC ACM example started.\n");
    

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now\n");

        app_usbd_enable();
        app_usbd_start();
    }
    
    rb_init(&write_buffer, usb_rb_tx_buffer, USB_WRITE_BUFFSIZE);
    rb_init(&read_buffer, usb_rb_tx_buffer, USB_READ_BUFFSIZE);

    return ret;
}

uint32_t usb_attempt_transmit(){
  //NRF_LOG_INFO("usb_attempt_transmit.\n");  
  if(!transmitDone){
    return NRF_ERROR_BUSY;
  }

  uint32_t ret = 0;
  
  transmitting = true;

  //NRF_LOG_RAW_INFO("rb_size %d\n", rb_size(&write_buffer));
  //uint64_t t1, t2;
  uint32_t read_amount;

  //t1 = systemTimeGetUs();
  //for(read_amount = 0; read_amount < USB_PACKET_SIZE && read_amount < rb_size(&write_buffer); read_amount++){
  //  m_tx_buffer[read_amount] = rb_at(&write_buffer, read_amount);
  //}

  //t2 = systemTimeGetUs();
  //NRF_LOG_RAW_INFO("OLD: %lld\n", t2 - t1);
  
  read_amount = USB_PACKET_SIZE < rb_size(&write_buffer) ? USB_PACKET_SIZE : rb_size(&write_buffer);
  rb_get_fast(&write_buffer, m_tx_buffer, read_amount);

  NRF_LOG_RAW_INFO("WRITING %d bytes\n", read_amount);
  if(read_amount > 0){
    ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, read_amount);
    if(ret == NRF_SUCCESS){
      transmitDone = false;
      
      uint32_t rbsize = rb_size(&write_buffer);
      
      rb_advance(&write_buffer, read_amount);
    }
  }

  transmitting = false;
  
  return ret;
}

// USB packets look like this:
// Metadata (1 byte)
// Data length (2 bytes)
// Data (X bytes)

// Write data to USB, metadata is information about packet, ex: robot id, type
bool usb_write(uint8_t metadata, uint8_t *data, uint16_t length){
  //NRF_LOG_RAW_INFO("SENDING USB DATA\n");
  if(metadata == USB_NONE){
    if(rb_put(&write_buffer, data, length)){
      return (usb_attempt_transmit() == NRF_SUCCESS);
    }
  }else if(rb_put_w(&write_buffer, data, length, metadata)){
    NRF_LOG_RAW_INFO("RB WRITE SUCCESSFUL\n");
    return (usb_attempt_transmit() == NRF_SUCCESS);
  }else{
    NRF_LOG_RAW_INFO("No space in buffer!\n");
  }
  return false;
}

// Write 2 buffers one after the other, useful for writing parameters
bool usb_write2(uint8_t metadata, uint16_t length1, const uint8_t * const data1, uint16_t length2, const uint8_t *const data2){
  // Write metadata | length1 + length2 | data1 | data2
  uint8_t total_length[2] = {MSB_16(length1 + length2), LSB_16(length1 + length2)};
  //NRF_LOG_RAW_INFO("Metadata: %d\n", metadata);
  if(rb_size(&write_buffer) + length1 + length2 + sizeof(uint16_t) + sizeof(metadata) < write_buffer.size - 1){
    if(rb_put(&write_buffer, &metadata, sizeof(metadata)) && // Write metadata
       rb_put(&write_buffer, total_length, 2) && // Write total length
       rb_put(&write_buffer, data1, length1) && // Write data1
       rb_put(&write_buffer, data2, length2) // Write data 2
       ){
      ret_code_t ret = usb_attempt_transmit();
      if(ret != NRF_SUCCESS){
        if (ret == NRF_ERROR_BUSY) {
          NRF_LOG_RAW_INFO("%08d usb_attempt_transmit() failed: resource busy\n", systemTimeGetUs());
        }else{
          NRF_LOG_ERROR("usb_attempt_transmit() failed: 0x%x != 0x%x", ret, NRF_ERROR_BUSY);
        }
      }

      //while(transmitting);

      return ret == NRF_SUCCESS;
    }else{
      NRF_LOG_RAW_INFO("Error transmitting 2 buffer packet!\n");
      return false;
    }
  }else{
    NRF_LOG_RAW_INFO("No space in buffer!\n");
    return false;
  }
}

volatile uint16_t usb_available(){
  return rb_size(&read_buffer);
}

uint16_t usb_get(){
  return rb_get(&read_buffer);
}
