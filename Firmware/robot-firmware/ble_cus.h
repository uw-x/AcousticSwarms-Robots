#ifndef _BLE_CUS_H_
#define _BLE_CUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gq.h"
#include "ble_config.h"


typedef struct ble_cus_s ble_cus_t; // Forward declaration

// Example BLE_CUS_DEF(m_dcs, dcs)
#define BLE_CUS_DEF(_name, _shorthand)\
  ble_cus_t _name; \
  void ble_ ## _shorthand ## _on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);\
  NRF_SDH_BLE_OBSERVER(_name ## _obs, BLE_HRS_BLE_OBSERVER_PRIO, ble_ ## _shorthand ## _on_ble_evt, &_name); \

//#define GET_CHAR_HANDLES(_shorthand, _cus, _handle) \
//      (((ble_ ## _shorthand ## _char_container_t *)_cus->characteristic_handles)->_handle)

typedef enum{
  BLE_CUS_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
  BLE_CUS_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
  BLE_CUS_EVT_TRANSFER_1KB,
  BLE_CUS_EVT_DISCONNECTED,
  BLE_CUS_EVT_CONNECTED,
} ble_cus_evt_type_t;

typedef struct{
  ble_cus_evt_type_t evt_type;                                  /**< Type of event. */
  uint32_t           bytes_transfered_cnt;   //!< Number of bytes sent during the transfer. */
} ble_cus_evt_t;

typedef void (*ble_cus_evt_handler_t) (ble_cus_t * p_cus, ble_cus_evt_t * p_evt);

/**@brief Custom Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct{
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  uint8_t                       initial_custom_value;          /**< Initial custom value */
  ble_srv_cccd_security_mode_t  custom_value_char_attr_md;     /**< Initial security level for Custom characteristics attribute */
} ble_cus_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_cus_s{
  uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
  uint8_t                       uuid_type;
  //ble_gatts_char_handles_t      status_char_handles;            /**< Handles related to the Custom Value characteristic. */
  //ble_gatts_char_handles_t      command_char_handles;
  nrf_ble_gq_t                  *p_gatt_queue;
  ble_gatts_char_handles_t*     characteristic_handles;
  ble_cus_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
  uint16_t                      max_payload_len;                //!< Maximum number of bytes that can be sent in one notification. */
  uint32_t                      kbytes_sent;                    //!< Number of kilobytes sent. */
  uint32_t                      bytes_sent;                     //!< Number of bytes sent. */
  uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
};


void ble_cus_on_gatt_evt(ble_cus_t * p_cus, nrf_ble_gatt_evt_t const * p_gatt_evt);
void ble_cus_on_cus_evt(ble_cus_t * p_cus_service, ble_cus_evt_t * p_evt);

bool ble_cus_send_notification(ble_cus_t * p_cus, const ble_gatts_char_handles_t *const handles, uint8_t * data, uint16_t length);
//uint32_t ble_cus_conn_notify(ble_cus_t * p_cus, ble_gatts_char_handles_t * handles, uint8_t data);
bool ble_cus_send_indication(ble_cus_t *p_cus, const ble_gatts_char_handles_t * const handles, uint8_t * data, uint16_t length);

void ble_cus_indication_confirm(void);
bool ble_cus_indication_in_progress(void);
 #endif