#ifndef _BLE_MANAGER_H_
#define _BLE_MANAGER_H_

#include "ble_srv_common.h"
#include "sensors.h"
#include "point.h"
#include "nav.h"

NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static uint16_t sequence_number = 0;
static volatile bool transmitDone;

void bleAdvertisingStart();
void bleInit(void);
void ble_reset_tx_state(void);

bool bleBufferHasSpace(uint8_t length);
void bleSendData(uint8_t * data, int length);
void send();
uint32_t bleGetRingBufferBytesAvailable(void);
void blePushSequenceNumber(void);
void resetBLE(void);

bool ble_manager_send_mic_data(uint8_t *packet, uint16_t length, bool use_indication);

uint16_t getSequenceNumber(void);
void ble_notify_recording_finished(void);
void bleSendSensorData(sensor_t *sensor);
bool ble_manager_send_metadata(uint8_t *metadata, uint16_t metadata_length);

void ble_send_xcorr_data(void);
void ble_add_xcorr_data(uint16_t xcorr_delay, nav_state_t *s1, nav_state_t *s2);
bool ble_xcorr_available(void);
void ble_indicate_milestone_reached(nav_state_t *current_state);
void ble_indicate_motion_correction(nav_state_t *current_state);
void ble_indicate_next_milestone(nav_state_t *current_state);

void ble_indication_buffer_process(void);
void ble_notification_buffer_process(void);

void ble_indicate_transition(uint8_t before, uint8_t after);
uint8_t ble_dump_nav_state(float *data, nav_state_t *state);
void ble_dump_xcorr_offset(uint8_t *data, uint16_t xcorr);
void ble_reset_xcorr_sequence_number(void);
void ble_reset_sequence_number(void);

bool ble_audio_stream_buffer_available(void);
bool ble_status_notification_buffer_available(void);
bool ble_status_indication_buffer_available(void);

void ble_notify_status(uint8_t status_code, uint8_t *status, uint8_t length);
void ble_indicate_status(uint8_t status_code, uint8_t *status, uint8_t length);
//void send(void);

#endif