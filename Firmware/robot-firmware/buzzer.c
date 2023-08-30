#include "buzzer.h"
#include "i2s.h"
#include "math.h"
#include "stereo.h"


bool buzzer_init(void){
  bool ok = i2s_init();
  //i2s_load_stream(chirp, SIGNAL_SIZE, 1);
  i2s_load_stream(warmup, WARMUP_SIZE, 1);
  i2s_start();
  return ok;
}

void buzzer_load_chirp(void){
  i2s_load_stream(chirp, SIGNAL_SIZE, 1); 
}

void buzzer_play_chirp(uint32_t nrepititions){
  i2s_load_stream(chirp, SIGNAL_SIZE, nrepititions);
  i2s_start();
}