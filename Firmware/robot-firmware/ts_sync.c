#include <stdint.h>
#include <stdbool.h>
#include "ts_sync.h"
#include "time_sync.h"
#include "nrf_log.h"

#include "ble_config.h"
#include "timers.h"
#include <math.h>
#include <stdlib.h>
#include "audio.h"


#define AUDIO_SYNC_DEBUG 0
#define AUDIO_SYNC_DEBUG_2 0

// TODO: Maybe reset these as well?
static bool biasInitialized     = false;
static uint64_t systemTimeBias  = 0;
static uint64_t syncTimeBias    = 0;
static int32_t prevTimerOffset  = 0;
static uint32_t offsetTolerance = 85;
static uint32_t tolerancePassed = 0;
static int64_t ticksAhead       = 0;
#define MAX_CYCLE_OFFSET 4000; //  @16MHz this is approx 60us

static int64_t toffp1, toffp2, toffp3;

void ts_sync_reset(void){
  // These should only be reset when timesync is restarted
  biasInitialized     = false;
  systemTimeBias  = 0;
  syncTimeBias    = 0;
  prevTimerOffset  = 0;
  offsetTolerance = 85;
  tolerancePassed = 0;
  ticksAhead = 0;
}

int64_t ts_sync_get_ticks_ahead(void){
  return ticksAhead;
}

void ts_sync_update_ticks_ahead(void){
  // 64 pdm clockes edges equates to 1 sample of PCM audio data
  // After 64 pdm clock edges a sample needs to be skipped

  // With f_pdm of 3.2MHz:
  // (64 clock cycles) / 3.2MHz = 20us
  // 20us on the 16MHz time sync clock is 320 ticks
  // Skip a 50khz sample after 320 ticks have accumulated

  // With f_pdm of 1MHz:
  // (64 clock cycles) / 1MHz = 64us
  // 64us on the 16MHz time sync clock is 1024 ticks
  // Skip a 15.625khz sample after 1024 ticks have accumulated

  // With f_pdm of 1MHz:
  // (64 clock cycles) / 2MHz = 32us
  // 32us on the 16MHz time sync clock is 512 ticks
  // Skip a 31250khz sample after 512 ticks have accumulated

  if (!ts_master() && audioStreamStarted()) {
  //if (!ts_master() && num_master_overflows() > MASTER_OVERFLOW_TRIGGER_COUNT) {
    uint64_t syncTimeTicks   = ts_timestamp_get_ticks_u64(6);
    uint64_t systemTimeTicks = systemTimeGetTicks();

    if (!biasInitialized) {
      biasInitialized = true;
      systemTimeBias  = systemTimeTicks;
      syncTimeBias    = syncTimeTicks;
      #if AUDIO_SYNC_DEBUG
        NRF_LOG_RAW_INFO("%08d [audio] pBias:%u lBias:%u\n", systemTimeGetMs(), syncTimeBias, systemTimeBias);
      #endif
    }

    // Subtract off bias
    int64_t relativeSystemTime = systemTimeTicks - systemTimeBias;
    int64_t relativeSyncTime   = syncTimeTicks - syncTimeBias;

    // Calculate offset
    int32_t timerOffset = (relativeSystemTime - relativeSyncTime) % TIME_SYNC_TIMER_MAX_VAL;

#if AUDIO_SYNC_DEBUG_2
    if(num_master_overflows() < 4){
      NRF_LOG_RAW_INFO("%08d [audio] Master overflows: %d [mod] timerOffset: %d [real] %d\n",
      systemTimeGetMs(), num_master_overflows(), timerOffset, relativeSystemTime - relativeSyncTime);
      }
#endif

    if (prevTimerOffset == 0) {
      prevTimerOffset = timerOffset;
    }

    toffp3 = toffp2;
    toffp2 = toffp1;
    toffp1 = timerOffset;

    // If there's an erroneous jump, then don't update ticksAhead
    // Likely hit this function as one timer was recently updated and the other hasn't
    //if (mod_abs(timerOffset - prevTimerOffset, TIME_SYNC_TIMER_MAX_VAL) < offsetTolerance) {
    if (abs(timerOffset - prevTimerOffset) < offsetTolerance) {
      // Only update ticksAhead if we are within tolerance for at LEAST 3 time sync packets
      // This gives the timers a chance to stabilize before updating ticksAhead
      if (tolerancePassed++ < 3) {
        offsetTolerance += 5;
      } else {
        ticksAhead      = timerOffset;
        prevTimerOffset = timerOffset;
        offsetTolerance = MAX_CYCLE_OFFSET;
      }
    } else {
      tolerancePassed = 0;
      offsetTolerance += 5; // Each time we fail to update, increase our acceptable tolerance
#if AUDIO_SYNC_DEBUG_2
      NRF_LOG_RAW_INFO("%08d [audio] offset:%d prevOffset:%d delta:%d\n",
        systemTimeGetMs(), timerOffset, prevTimerOffset, abs(timerOffset - prevTimerOffset));
#endif
    }

//    // Sometimes flag gets permanently set to false for some reason and this causes stalling.
//    uint64_t arr[4] = {toffp1, toffp2, toffp3, timerOffset};  
//    uint64_t mn = toffp1, mx = toffp1;
//    for(int i = 1; i < 4; i++){
//      mn = (mn < arr[i]) ? mn : arr[i];
//      mx = (mx > arr[i]) ? mx : arr[i];
//    }
//    if(abs(mn - mx) < 1){
//      ts_reset_flag();
//#if AUDIO_SYNC_DEBUG_2
//      NRF_LOG_RAW_INFO("STALL EXITED\n");
//#endif
//    }

#if AUDIO_SYNC_DEBUG_2
    NRF_LOG_RAW_INFO("%08d [audio] p:%lld h:%lld o:%d t:%d\n",
      systemTimeGetMs(), relativeSyncTime, relativeSystemTime, timerOffset, ticksAhead);
    NRF_LOG_RAW_INFO("%08d [audio] pBias:%lld hBias:%lld pTime:%lld hTime:%lld\n",
      systemTimeGetMs(), syncTimeBias, systemTimeBias, syncTimeTicks, systemTimeTicks);
#endif
  }
}