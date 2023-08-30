#ifndef _USB_H_
#define _USB_H_

#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "boards.h"


#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

#define METADATA_SIZE 3 // 1 type + 2 length

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

typedef enum {
  USB_NONE = 0,
  USB_MIC_DATA,
  USB_BLE_COMMAND,
  USB_BLE_INFO,
  USB_BLE_DEVICE_CONNECTED,
  USB_BLE_DEVICE_DISCONNECTED,
  USB_BLE_RECORDING_FINISHED,
  USB_INFO,
  USB_SENSOR_DATA,
  USB_BLE_DATABASE_DISCOVERY_COMPLETE,
  USB_BLE_STATUS_UPDATE,
} usb_packet_type_t;

typedef enum {
  BLE_GATT_REQUEST = 0,
  BLE_SCAN_START,
  BLE_SCAN_STOP,
  BLE_TX_AWAIT_FINISHED,
} usb_ble_command_type_t;

typedef enum {
  BLE_TX_FINISHED = 0,
} usb_ble_info_t;

typedef enum {
  BLE_START_NOTIFICATIONS = 0,
  BLE_STOP_NOTIFICATIONS,
  BLE_WRITE_CHARACTERISTIC,
  BLE_READ_CHARACTERISTIC,
} usb_ble_gatt_request_type_t;

//typedef enum {
//  USB_BLE_SCAN_START = 0,
//  USB_BLE_ENABLE_STREAM,
//  USB_BLE_AWAIT_CONNECTION,
//  USB_BLE_ENABLE_MASTER,
//  USB_BLE_DISABLE_STREAM,
//  USB_BLE_SCAN_STOP,
//  USB_BLE_START_CLICK,
//  USB_BLE_STOP_CLICK,
//} usb_command_type_t;


static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);


ret_code_t usb_init();
bool usb_write(uint8_t metadata, uint8_t *data, uint16_t length);
bool usb_write2(uint8_t metadata, uint16_t length1, const uint8_t * const data1, uint16_t length2, const uint8_t *const data2);

volatile uint16_t usb_available();
uint16_t usb_get();

#endif