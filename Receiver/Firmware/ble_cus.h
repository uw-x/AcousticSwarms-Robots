#ifndef _BLE_CUS_H_
#define _BLE_CUS_H_


#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gq.h"
#include "ble_db_discovery.h"


// This was written by following:
// https://github.com/bjornspockeli/custom_ble_service_example

// Generate UUIDs here:
// https://www.uuidgenerator.net/version4

// Shio Mic Stream Service
// 47ea50d7-e4a0-4e55-8252-0afcd3246970
#define CUSTOM_SERVICE_UUID_BASE         {0x70, 0x69, 0x24, 0xD3, 0xFC, 0x0A, 0x82, 0x52, \
                                          0x4E, 0x55, 0xE4, 0xA0, 0xD7, 0x50, 0xEA, 0x47}

// When adding/changing a characteristic, turn bluetooth off and on in your device's settings
#define CUSTOM_SERVICE_UUID               0x1400
#define CUSTOM_VALUE_CHAR_UUID            0x1401
#define MIC_CHAR_UUID                     0x1402
#define CONTROL_CHAR_UUID                 0x1403

#define BLE_CUS_DEF(_name) \
  static ble_cus_t _name;
  //NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_HRS_BLE_OBSERVER_PRIO, ble_cus_on_ble_evt, &_name)

typedef struct ble_cus_s ble_cus_t; // Forward declaration

#define BLE_REQ_TYPE_WRITE 1
#define BLE_REQ_TYPE_CCCD 2

typedef enum
{
  BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
  BLE_CUS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
  BLE_CUS_EVT_TRANSFER_1KB,
  BLE_CUS_EVT_DISCONNECTED,
  BLE_CUS_EVT_CONNECTED,
} ble_cus_evt_type_t;

typedef struct
{
  ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
  uint32_t           bytes_transfered_cnt;   //!< Number of bytes sent during the transfer. */
} ble_cus_evt_t;

typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  uint8_t                       initial_custom_value;          /**< Initial custom value */
  ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

typedef struct{
  uint16_t uuid;
  uint16_t value_handle;
  uint16_t cccd_handle;
  ble_gatt_char_props_t props;
} ble_cus_characteristic_t;

typedef struct{
  uint16_t uuid;
  uint8_t n_characteristics;
  ble_cus_characteristic_t *characteristics;
} ble_cus_service_t;

typedef struct{
  ble_gap_addr_t addr;
  uint16_t conn_handle;
  ble_cus_service_t *services;
  uint8_t n_services;
} ble_device_t;

typedef void (*ble_cus_notification_handler_t) (uint8_t *handler, uint8_t *data);

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s{
  uint8_t                       uuid_type;
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  uint16_t                      max_payload_len;                //!< Maximum number of bytes that can be sent in one notification. */
  uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
  nrf_ble_gq_t                  *p_gatt_queue;
  uint8_t                       device_count;
  ble_device_t                  *ble_device_list[NRF_SDH_BLE_CENTRAL_LINK_COUNT];
};

uint32_t ble_cus_init(nrf_ble_gq_t *gq);

uint32_t cccd_configure(uint16_t conn_handle, uint16_t uuid, bool notification_enable, bool bypass);
bool ble_cus_write_char(uint16_t conn_handle, uint16_t uuid, uint8_t *data, uint16_t length);
void ble_cus_write_mic_data(uint16_t conn_handle, const uint8_t* const data, uint16_t length);

uint32_t ble_cus_add_device(ble_gap_addr_t dev_addr, uint16_t conn_handle);
uint32_t ble_cus_remove_device(uint16_t conn_handle, uint8_t reason);
uint32_t ble_cus_attempt_transmit(bool bypass);
bool ble_write_buffer_empty();

void ble_cus_handle_notification(const ble_gattc_evt_t* const p_ble_evt);
void ble_cus_db_disc_evt(ble_db_discovery_evt_t * p_evt);
void ble_cus_on_device_discovery_done();
uint8_t ble_get_device_count(void);

void ble_init(void);
uint32_t scan_time(void);

bool ble_cus_scan_stopped(void);
ble_cus_t *ble_cus_get_service(void);
void ble_cus_write_tx_finished(void);

void scan_start(void);
void scan_stop(void);

#endif