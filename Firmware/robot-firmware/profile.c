#include "profile.h"
#include "audio.h"
#include "ts_tasks.h"
#include "nrf_log.h"

profile_t profiles[NUM_PROFILES];

void _profile_tof(const uint8_t* data, uint16_t length){
    audio_params_t params;
    params.div_clk = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3];
    params.decimation_factor = data[4];
    params.ratio = data[5];
    params.left_gain = data[6];
    params.right_gain = data[6];
    params.mode = 0; // Stereo
    params.use_compression = 0; // No compression

    uint8_t is_spk = data[7];
    uint16_t period = (data[8] << 8) + data[9];
    uint32_t begin_interval = (data[10] << 24) + (data[11] << 16) + (data[12] << 8) + data[13];
    uint32_t end_interval = (data[14] << 24) + (data[15] << 16) + (data[16] << 8) + data[17];
    if(end_interval == 0){
      end_interval = 1LL << 31;
    }
    
    ts_tasks_set_role(TS_ROLE_SLAVE);
    audio_params_update(&params);

    ts_sync_task_add(TS_TASK_MIC_START, 0);
    
    ts_sync_task_add_recurring(TS_TASK_MIC_RECORD, period, begin_interval, end_interval);

    if(is_spk){
      ts_sync_task_add_recurring(TS_TASK_SPK_START, period, begin_interval, end_interval);
    }

    NRF_LOG_RAW_INFO("[profile] LOADED PROFILE: TOF (param length: %d)\n", length);
}

void profile_init(void){
  profiles[PROFILE_TOF] = (profile_t) _profile_tof;
}

void profile_load(uint8_t id, const uint8_t *params, uint16_t length){
  profiles[id](params, length);
}