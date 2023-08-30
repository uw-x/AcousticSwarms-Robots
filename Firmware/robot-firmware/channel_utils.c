#include "channel_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "nrf_log.h"
#include "timers.h"

#ifdef USE_FFT_CORR
  MIC_TYPE in1[CHANNEL_LEN*2];
  //MIC_TYPE chirp_buffer[CHANNEL_LEN];
  MIC_TYPE left_buffer[CHANNEL_LEN];
  MIC_TYPE right_buffer[CHANNEL_LEN];

  float32_t chirp_channel[CHANNEL_LEN*2];
  float32_t left_result[CHANNEL_LEN];
  float32_t right_result[CHANNEL_LEN];
  float32_t left_power[CHANNEL_LEN];
  float32_t right_power[CHANNEL_LEN];
#else
  MIC_TYPE cross_vector[MAX_CORR_LEN];
#endif

uint32_t estimate_offsets[CHIRP_NUM];
uint32_t chirp_index;

uint32_t xcorr_time;
uint32_t channel_time; 
uint32_t begin_time;


int32_t coarse_index = 0;


int16_t INIT_FLAG = 0;
uint32_t last_fine_grained_index = 0;


uint32_t track_begin_index = 0;
uint32_t track_end_index = 0;
float32_t left_noise = 0;
float32_t right_noise = 0; 

void reset_tracking(){
  INIT_FLAG = 0;
  last_fine_grained_index = 0;
  coarse_index = 0;
  track_begin_index = 0;
  track_end_index = 0;
}

void print_q15(q15_t* arr, uint16_t lens){
    uint32_t i;
    for(i= 0; i < lens; ++i ){
      if(2*i == lens){
        NRF_LOG_RAW_INFO("\n");
      }
      NRF_LOG_RAW_INFO("%d, ", arr[i]);
    }
    NRF_LOG_RAW_INFO("\n");
}

void print_f32(float32_t *arr, uint16_t lens){
    uint32_t i;
    for(i= 0; i < lens; ++i ){
      if(2*i == lens){
        NRF_LOG_RAW_INFO("\n");
      }
      NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ", ", NRF_LOG_FLOAT(arr[i]) );
    }
    NRF_LOG_RAW_INFO("\n");
}

void real_to_complex_f32(const float32_t * real_input, float32_t* complex_output, uint16_t size_input){
  uint16_t i;
  for(i = 0; i < size_input; ++i ){
    complex_output[2*i] = real_input[i];
    complex_output[2*i + 1] = 0; 
  }
}

void real_to_complex_q15(const q15_t * real_input, q15_t* complex_output, uint16_t size_input){
  uint16_t i;
  for(i = 0; i < size_input; ++i ){
    complex_output[2*i] = real_input[i];
    complex_output[2*i + 1] = 0; 
  }
}


float32_t Complex_power(float32_t real, float32_t imag){
    return real*real + imag*imag;
}


void norm_cmplx_f32(float32_t* X, uint16_t input_size){
    uint16_t i;
    float32_t x_real;
    float32_t x_imag;
    float32_t x_pow;

    for(i = 0; i < input_size; ++i ){
        x_real = X[2*i];
        x_imag = X[2*i + 1];
        x_pow = Complex_power(x_real, x_imag);
        if(x_pow == 0){
          X[2*i] = 0;
          X[2*i + 1] = 0; 
        }
        else{
          X[2*i] = x_real/x_pow;
          X[2*i + 1] = -x_imag/x_pow;
        }
    }
}

void init_chirp_fft(){
    memset(chirp_channel, 0, sizeof(float32_t)*CHANNEL_LEN*2);

    real_to_complex_f32(chirp, chirp_channel, CHIRP_LEN);

    arm_cfft_f32(&arm_cfft_sR_f32_len2048, chirp_channel, FFT_FLAG, m_do_bit_reverse);

    norm_cmplx_f32(chirp_channel, CHANNEL_LEN);
    arm_scale_f32(chirp_channel, MULTI_FACTOR, chirp_channel, CHANNEL_LEN*2);
}

void real_to_complex(const MIC_TYPE * real_input, MIC_TYPE* complex_output, uint16_t size_input){
  uint16_t i;
  for(i = 0; i < size_input; ++i ){
    complex_output[2*i] = real_input[i];
    complex_output[2*i + 1] = 0; 
  }
}

void complex_to_real(const MIC_TYPE * complex_input, MIC_TYPE* real_output, uint16_t size_real){
  uint16_t i;

  for(i = 0; i < size_real; ++i ){
    real_output[i] = complex_input[2*i] ;
  }
}

void complex_to_real_f32(const float32_t * complex_input, float32_t* real_output, uint16_t size_real){
  uint16_t i;
  for(i = 0; i < size_real; ++i ){
    real_output[i] = complex_input[2*i] ;
  }
}


void estimate_H(const float32_t* X, MIC_TYPE* Y, uint16_t fft_begin, uint16_t fft_end){
    uint16_t  i = 0;

    float32_t x_real = 0;
    float32_t x_imag = 0;
    float32_t y_real = 0;
    float32_t y_imag = 0;
    float32_t r_real = 0;
    float32_t r_imag = 0;
    

    for(i = 0; i < CHANNEL_LEN ; ++i){
        if(i >= fft_begin && i < fft_end){
            x_real = X[2*i];
            x_imag = X[2*i + 1];
            y_real = (float32_t)Y[2*i];
            y_imag = (float32_t)Y[2*i + 1];

            r_real = (x_real*y_real - x_imag*y_imag);
            r_imag = (y_imag*x_real + y_real*x_imag);
            if(abs(r_real) >=  32767 || abs(r_imag) >=  32767){
                NRF_LOG_RAW_INFO("Warining Overflow\n");
              }
            #ifdef DEBUG_CHANNEL
              if(abs(r_real) >=  32767 || abs(r_imag) >=  32767){
                NRF_LOG_RAW_INFO("Warining Overflow\n");
              }
            #endif

            Y[2*i] = (MIC_TYPE) (r_real);
            Y[2*i+1] = (MIC_TYPE) (r_imag);
            //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "," NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(Y[2*i]) ,NRF_LOG_FLOAT(Y[2*i+1]) );
            //NRF_LOG_RAW_INFO(" => %d, %d\n", Y[2*i] ,Y[2*i+1] );
        }
        else{
            Y[2*i] = 0;
            Y[2*i+1] = 0;
        }
    }
}



void my_mag_q15(const q15_t * pSrc, float32_t * pDst, uint32_t numSamples){
    uint32_t i = 0;
    float32_t r1;
    float32_t r2;
    for(i = 0; i < numSamples; ++i){
        r1 = (float32_t) pSrc[2*i];
        r2 = (float32_t) pSrc[2*i + 1];
        *(pDst+i) = (r1 * r1) + (r2 * r2);
    }
}

void my_mag_q15_2(const q15_t * pSrc, float32_t * pDst, uint32_t begin_index, uint32_t end_index){
    uint32_t i = 0;
    float32_t r1;
    float32_t r2;
    for(i = begin_index; i < end_index; ++i){
        r1 = (float32_t) pSrc[2*i];
        r2 = (float32_t) pSrc[2*i + 1];
        *(pDst+i) = (r1 * r1) + (r2 * r2);
    }
}

float32_t my_mag_sum(const q15_t * pSrc, uint32_t numSamples){
    uint32_t i = 0;
    float32_t r1;
    float32_t r2;
    float32_t value = 0;
    for(i = NOISE_BEGIN; i < NOISE_BEGIN + numSamples; ++i){
        r1 = (float32_t) pSrc[2*i];
        r2 = (float32_t) pSrc[2*i + 1];
        value += ((r1 * r1) + (r2 * r2))/numSamples;
    }

    return value;
}

void channel_estimation_freq(const MIC_TYPE * rx, float32_t* channel_result, uint16_t sig_len, uint16_t left_right){
    // CHANNEL_LEN -- 2048
    uint16_t i;
    uint32_t final_index;
    float32_t delta_f = (float32_t)FS/CHANNEL_LEN;
    uint16_t fft_begin = (uint16_t) round(BW_BEGIN/delta_f);
    uint16_t fft_end = (uint16_t) round(BW_END/delta_f);
    
    memset(in1, 0, sizeof(MIC_TYPE)*CHANNEL_LEN*2);
    memset(channel_result, 0, sizeof(float32_t)*CHANNEL_LEN);

    real_to_complex(rx, in1, sig_len);

    my_fft(p_struct_2048, in1, FFT_FLAG, m_do_bit_reverse);
    
    estimate_H(chirp_channel, in1, fft_begin, fft_end);
    //print_q15(in1, CHANNEL_LEN*2);

    my_fft(p_struct_2048, in1, IFFT_FLAG, m_do_bit_reverse);

    //print_q15(in1, CHANNEL_LEN*2);

    my_mag_q15((q15_t*)in1, channel_result, CHANNEL_RESULT_LEN);
    
    if(left_right){
       right_noise = 25*my_mag_sum((q15_t*)in1, NOISE_LEN);
    }
    else{
       left_noise = 25*my_mag_sum((q15_t*)in1, NOISE_LEN);
    }
    //print_f32(channel_result, CHANNEL_RESULT_LEN);
    //print_f32(channel_result, 600);
}

uint32_t channel_estimation_freq2(const MIC_TYPE * rx, float32_t* channel_result, uint16_t sig_len){
    // CHANNEL_LEN -- 2048
    uint16_t i;

    float32_t delta_f = (float32_t)FS/CHANNEL_LEN;
    uint16_t fft_begin = (uint16_t) round(BW_BEGIN/delta_f);
    uint16_t fft_end = (uint16_t) round(BW_END/delta_f);

    memset(in1, 0, sizeof(MIC_TYPE)*FFT_CORR_LEN*2);
    memset(channel_result, 0, sizeof(float32_t)*CHANNEL_LEN);
    
    real_to_complex(rx, in1, sig_len);

    my_fft(p_struct_2048, in1, FFT_FLAG, m_do_bit_reverse);

    estimate_H(chirp_channel, in1, fft_begin, fft_end);
   
    my_fft(p_struct_2048, in1, IFFT_FLAG, m_do_bit_reverse);

    my_mag_q15_2((q15_t*)in1, channel_result, track_begin_index, track_end_index);
    //print_f32(channel_result, 600);
}

bool check_peak(float32_t* arr, uint32_t idx)
{ 
    return (arr[idx] > arr[idx+1]) && (arr[idx] > arr[idx+2]) && (arr[idx] > arr[idx-1]) && (arr[idx] > arr[idx-2]);
}

bool check_sharp_peak(float32_t* arr, uint32_t idx)
{ 
    return (arr[idx] > arr[idx+1]) && (arr[idx+1] > arr[idx+2]) && (arr[idx] > arr[idx-1]) && (arr[idx] > arr[idx-2]);
}

bool check_small_peak(float32_t* arr, uint32_t idx)
{ 
    return (arr[idx] > arr[idx+1]) && (arr[idx] > arr[idx-1]);
}



uint32_t Single_mic_track(uint16_t I_am_master){
    uint32_t max_index1 = 0;
    float32_t max_value1 = 0;
    float32_t current_value;
    uint32_t i = 0;

    if(I_am_master){
        arm_max_f32(left_result+track_begin_index, track_end_index - track_begin_index, &max_value1, &max_index1);
        return max_index1 + track_begin_index;
    }
    
    arm_max_f32(left_result+track_begin_index, track_end_index - track_begin_index, &max_value1, &max_index1);
    float32_t peak_value = max_value1*TRACK_THRESHOLD;
    uint32_t peak_index = max_index1;

    for(i = track_begin_index + 2; i < track_end_index - 1; ++i){
        if(left_result[i] > left_result[i - 1] && left_result[i] > left_result[i + 1] && left_result[i] > left_result[i-2] && left_result[i] > left_result[i+2] && left_result[i] >= peak_value){
          //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER ", with index %d\n", NRF_LOG_FLOAT(channel_result[i]), i );
          peak_index = i;
          break;
        }
    }
    
    return peak_index ;

}



uint32_t Dual_mic_track(uint16_t I_am_master){
    uint32_t max_index1 = 0;
    float32_t max_value1 = 0;
    uint32_t max_index2 = 0;
    float32_t max_value2 = 0;

    float32_t current_value;
    uint32_t i = 0;
    uint32_t j = 0;

    if(I_am_master){
        arm_max_f32(left_result+track_begin_index, track_end_index - track_begin_index, &max_value1, &max_index1);
        arm_max_f32(right_result+track_begin_index, track_end_index - track_begin_index, &max_value2, &max_index2);
        return (max_index1 + max_index2) + 2*track_begin_index;
    }
    return 0; 
    memset(left_power, 0, sizeof(float32_t)*CHANNEL_LEN);
    memset(right_power, 0, sizeof(float32_t)*CHANNEL_LEN);
    //print_f32(left_result, CHANNEL_RESULT_LEN);
    //print_f32(right_result, CHANNEL_RESULT_LEN);

    arm_conv_f32(left_result, CHANNEL_RESULT_LEN, avg_filter, 9, left_power);
    arm_conv_f32(right_result, CHANNEL_RESULT_LEN, avg_filter, 9, right_power);


    float32_t* left_power2 = left_power + 4;
    float32_t* right_power2 = right_power + 4;

    /*
    arm_max_f32(left_result, CHANNEL_RESULT_LEN, &max_value1, &max_index1);
    arm_max_f32(right_result, CHANNEL_RESULT_LEN, &max_value2, &max_index2);
    float32_t peak_value1 = max_value1*PEAK_THRESHOLD;
    float32_t peak_value1_low = max_value1*PEAK_THRESHOLD2;
    float32_t peak_value2 = max_value2*PEAK_THRESHOLD;
    float32_t peak_value2_low = max_value2*PEAK_THRESHOLD2;
    */

    uint32_t another_index = 0;
    //NRF_LOG_RAW_INFO("track_begin_index= %d, %d, \n", track_begin_index, track_end_index);

    for(i = track_begin_index + 4; i < track_end_index - 3; ++i){
        //NRF_LOG_RAW_INFO("track_begin_index= %d, \n", i );
        //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "ratio \n", NRF_LOG_FLOAT(left_power2[i-1]/left_power2[i-5]));
        //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "ratio_up \n", NRF_LOG_FLOAT(left_power2[i+4]/left_power2[i]));
        //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "ratio2 \n", NRF_LOG_FLOAT(right_power2[i-1]/right_power2[i-5]));
        //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "ratio2_up \n", NRF_LOG_FLOAT(right_power2[i+4]/right_power2[i]));

        //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "ratio22 \n", NRF_LOG_FLOAT(right_power2[i-2]/right_power2[i-6]));
        if(check_sharp_peak(left_result, i) && 
          (left_power2[i-1]/left_power2[i-5] > RATIO_THRESHOLD || left_power2[i-2]/left_power2[i-6] > RATIO_THRESHOLD) 
          && left_power2[i+4]/left_power2[i] < RATIO_THRESHOLD_UP
          && left_result[i] > left_noise){
              for(j = i-DUAL_RANGE; j <= i + DUAL_RANGE; ++j){
                  if(check_peak(right_result, j) && right_result[j] > right_noise){
                          another_index = j;
                          break;
                      }
              }
              NRF_LOG_RAW_INFO("tracking left: i, another= %d, %d, \n", i, another_index);
              if(another_index > 0){
                return (another_index + i);
              }
              else{
                return i*2;
              }
        }

        if(check_sharp_peak(right_result, i) 
        && (right_power2[i-1]/right_power2[i-5] > RATIO_THRESHOLD || right_power2[i-2]/right_power2[i-6] > RATIO_THRESHOLD) 
        && right_power2[i+4]/right_power2[i] < RATIO_THRESHOLD_UP
        && right_result[i] > right_noise){
              for(j = i-DUAL_RANGE; j <= i + DUAL_RANGE; ++j){
                  if(check_peak(left_result, j) && left_result[j] > left_noise){
                          another_index = j;
                          break;
                      }
              }
              NRF_LOG_RAW_INFO("tracking left: i, another= %d, %d, \n", i, another_index);
              if(another_index > 0){
                return (another_index + i) ;
              }
              else{
                return i*2;
              }
        }
    }
    
    return 0; //last_fine_grained_index*2;

}



uint32_t Single_mic_estimation(uint16_t I_am_master)
{ 
    
    float32_t current_value;
    uint32_t i = 0;

    uint32_t max_index1 = 0;
    float32_t max_value1 = 0;
    
    uint32_t new_index =  0;    

    // master estimation
    if(I_am_master){
        arm_max_f32(left_result, CHANNEL_RESULT_LEN, &max_value1, &max_index1);
        //float32_t direct_path_height = max_value1*PEAK_THRESHOLD;
        new_index = max_index1;
        return new_index;
    }
  
    // slaver estimation
    arm_max_f32(left_result, CHANNEL_RESULT_LEN, &max_value1, &max_index1);
    float32_t direct_path_height1 = max_value1*PEAK_THRESHOLD;
    new_index = max_index1;

    for(i = 2; i < max_index1; ++i){
        current_value = left_result[i];
        if(current_value > left_result[i-1] && 
            current_value > left_result[i+1] && 
            current_value > left_result[i-2] && 
            current_value > left_result[i+2] &&
            current_value > direct_path_height1){
              new_index = i;
              break;
        }
    }

    NRF_LOG_RAW_INFO("new_index = %d, \n", new_index);

    return new_index;

}

uint32_t Dual_mic_estimation(uint16_t I_am_master){
  uint32_t max_index1 = 0;
  float32_t max_value1 = 0;
  uint32_t max_index2 = 0;
  float32_t max_value2 = 0;

  uint32_t i = 0;
  uint32_t j = 0;
  uint32_t other_index =0 ;
  uint32_t power_coarse_id = 0;
  //print_f32(left_result, CHANNEL_RESULT_LEN);
  //print_f32(right_result, CHANNEL_RESULT_LEN);
  if(I_am_master){
        arm_max_f32(left_result, CHANNEL_RESULT_LEN, &max_value1, &max_index1);
        arm_max_f32(right_result, CHANNEL_RESULT_LEN, &max_value2, &max_index2);
        //return (max_index1 + max_index2)/2 + track_begin_index;
        return max_index1 + max_index2 + track_begin_index * 2;
  }

  memset(left_power, 0, sizeof(float32_t)*CHANNEL_LEN);
  memset(right_power, 0, sizeof(float32_t)*CHANNEL_LEN);
  //print_f32(left_result, CHANNEL_RESULT_LEN);
  //print_f32(right_result, CHANNEL_RESULT_LEN);

  arm_conv_f32(left_result, CHANNEL_RESULT_LEN, avg_filter, 9, left_power);
  arm_conv_f32(right_result, CHANNEL_RESULT_LEN, avg_filter, 9, right_power);


  float32_t* left_power2 = left_power + 4;
  float32_t* right_power2 = right_power + 4;
  //print_f32(left_power2, CHANNEL_RESULT_LEN);
  //print_f32(right_power2, CHANNEL_RESULT_LEN);
  for(i = SELF_BEGIN; i < CHANNEL_RESULT_LEN - 20; ++i){
      //NRF_LOG_RAW_INFO("i= %d \n", i);
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio \n", NRF_LOG_FLOAT(left_power2[i-1]/left_power2[i-5]));
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio2 \n", NRF_LOG_FLOAT(right_power2[i-1]/right_power2[i-5]));
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio2 \n", NRF_LOG_FLOAT(left_power2[i+4]/left_power2[i]));
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio2 \n", NRF_LOG_FLOAT(right_power2[i+4]/right_power2[i]));
      if(check_sharp_peak(left_power2, i) && left_power2[i] > left_noise && left_power2[i-1]/left_power2[i-5] > 2 && left_power2[i+5]/left_power2[i] < 8){
        power_coarse_id = i;
        break;
      }
      if(check_sharp_peak(right_power2, i) && right_power2[i] > right_noise && right_power2[i-1]/right_power2[i-5] > 2 && right_power2[i+5]/right_power2[i] < 8){
        power_coarse_id = i;
        break;
      }
  }
  NRF_LOG_RAW_INFO("power_coarse_id = %d, \n", power_coarse_id);
  if(power_coarse_id == 0)
  { 
      return 0;
  }
  //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "right_noise \n", NRF_LOG_FLOAT(right_noise));
  //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "left_noise \n", NRF_LOG_FLOAT(left_noise));
  for( i = power_coarse_id - 8; i <= power_coarse_id + 8; ++i){
      //NRF_LOG_RAW_INFO("i= %d \n", i);
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio \n", NRF_LOG_FLOAT(left_power2[i-1]/left_power2[i-5]));
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio_up \n", NRF_LOG_FLOAT(left_power2[i+4]/left_power2[i]));
      //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio2 \n", NRF_LOG_FLOAT(right_power2[i-1]/right_power2[i-5]));
      if(check_sharp_peak(left_result, i) 
      && (left_power2[i-1]/left_power2[i-5] > RATIO_THRESHOLD || left_power2[i-2]/left_power2[i-6] > RATIO_THRESHOLD)
      && left_power2[i+4]/left_power2[i] < RATIO_THRESHOLD_UP ){
          for(j = i - DUAL_RANGE; j <= i + DUAL_RANGE; ++j){
              //NRF_LOG_RAW_INFO("j= %d \n", j);
              //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "right_result \n", NRF_LOG_FLOAT(right_result[j]));
              //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER " ratio2 \n", NRF_LOG_FLOAT(right_power2[i-1]/right_power2[i-5]));
              if(check_peak(right_result, j) && right_result[j] > right_noise){
                  other_index = j;
                  break;
              }
          }
          NRF_LOG_RAW_INFO("left other_index = %d, %d \n", other_index, i);
          if(other_index == 0){
              return i * 2;
          }
          else{
              //return (i+other_index)/2;
              return i+other_index;
          }

      }
      if(check_sharp_peak(right_result, i) && 
      (right_power2[i-1]/right_power2[i-5] > RATIO_THRESHOLD || right_power2[i-2]/right_power2[i-6] > RATIO_THRESHOLD)
      && right_power2[i+4]/right_power2[i] < RATIO_THRESHOLD_UP ){
         for(j = i - DUAL_RANGE; j <= i + DUAL_RANGE; ++j){
              if(check_peak(left_result, j) && left_result[j] > left_noise){
                  other_index = j;
                  break;
              }
          }
          NRF_LOG_RAW_INFO("left other_index = %d, %d \n", other_index, i);

          if(other_index == 0){
              return i * 2;
          }
          else{
              //return (i+other_index)/2;
              return i+other_index;
          }
      }
  }
  return power_coarse_id*2;

  //print_f32(left_power+4, CHANNEL_RESULT_LEN);

  //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "left_noise \n", NRF_LOG_FLOAT(left_noise));
  //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "right_noise \n", NRF_LOG_FLOAT(right_noise));
}
/*
uint32_t Dual_mic_estimation(uint16_t I_am_master)
{ 
    
    float32_t current_value;
    uint32_t i = 0;

    uint32_t max_index1 = 0;
    float32_t max_value1 = 0;

    uint32_t max_index2 = 0;
    float32_t max_value2 = 0;
    
    uint32_t end_index = 0;

    uint32_t new_index =  0;

    float32_t* other_channel;
    

    // master estimation
    if(I_am_master){
        arm_max_f32(left_result, CHANNEL_RESULT_LEN, &max_value1, &max_index1);
        float32_t direct_path_height = max_value1*PEAK_THRESHOLD;
        new_index = max_index1;
        return new_index;
    }
    
    
    // slaver estimation
    arm_max_f32(left_result, CHANNEL_RESULT_LEN, &max_value1, &max_index1);
    arm_max_f32(right_result, CHANNEL_RESULT_LEN, &max_value2, &max_index2);
    
    float32_t direct_path_height1 = max_value1*PEAK_THRESHOLD;
    float32_t direct_path_height2 = max_value2*PEAK_THRESHOLD;
    float32_t other_low = 0;
    //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "direct_path_height1 \n", NRF_LOG_FLOAT(direct_path_height1));
    //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "direct_path_height2 \n", NRF_LOG_FLOAT(direct_path_height2));
    end_index = (max_index1 > max_index2)? max_index2 + DUAL_RANGE: max_index1 + DUAL_RANGE; 
    //NRF_LOG_RAW_INFO("end_index = %d, %d, %d,\n ", max_index1, max_index2, end_index);

    for(i = 5; i < end_index; ++i){
        current_value = left_result[i];
        if(current_value > left_result[i-1] && 
            current_value > left_result[i+1] && 
            current_value > left_result[i-2] && 
            current_value > left_result[i+2] &&
            current_value > direct_path_height1){
              new_index = i;
              other_channel = right_result;
              other_low = max_value2*PEAK_THRESHOLD2;
              break;
        }
        current_value = right_result[i];
        if(current_value > right_result[i-1] && 
            current_value > right_result[i+1] && 
            current_value > right_result[i-2] && 
            current_value > right_result[i+2] &&
            current_value > direct_path_height2){
              new_index = i;
              other_channel = left_result;
              other_low = max_value1*PEAK_THRESHOLD2;
              break;
        }
    }
    NRF_LOG_RAW_INFO("new_index = %d, \n", new_index);

    if(new_index == 0){
        return max_index1;
    }
    //NRF_LOG_RAW_INFO(NRF_LOG_FLOAT_MARKER "Threshold \n", NRF_LOG_FLOAT(other_low));
    //print_f32(other_channel, 600);

    uint32_t new_index2 =  0;
    float32_t new_peak2 =  0;

    for(i = new_index - DUAL_RANGE; i <= new_index + DUAL_RANGE; ++i){
        current_value = other_channel[i];
        if(current_value > other_channel[i-1] && 
            current_value > other_channel[i+1] && 
            current_value > other_channel[i-2] && 
            current_value > other_channel[i+2] &&
            current_value > other_low){
              if(current_value > new_peak2){
                  new_peak2 = current_value;
                  new_index2 = i;
              }
         }
    }
    NRF_LOG_RAW_INFO("new_index2 = %d, \n", new_index2);
    if(new_index2 == 0){
        return new_index;
    }
    else{
        return (new_index + new_index2)/2;
    }
}
*/ 
void detection_phase_left(MIC_TYPE* recv_signal, uint16_t sig_len){
    uint32_t fine_grained_index = 0;

    memcpy(left_buffer, recv_signal + FIX_COARSE_INDEX , sizeof(MIC_TYPE)*CHANNEL_LEN);
    
    if(INIT_FLAG){
           if(last_fine_grained_index <= CONTINUE_TRACKING_RANGE) track_begin_index = 0;
           else track_begin_index = last_fine_grained_index - CONTINUE_TRACKING_RANGE;

           if(last_fine_grained_index +  CONTINUE_TRACKING_RANGE >= CHANNEL_LEN) track_end_index = CHANNEL_LEN - 1;
           else track_end_index = last_fine_grained_index +  CONTINUE_TRACKING_RANGE;

           channel_estimation_freq(left_buffer, left_result, CHIRP_LEN, 0);
     }
     else{
           channel_estimation_freq(left_buffer, left_result, CHIRP_LEN, 0);
     }

}

void detection_phase_right(MIC_TYPE* recv_signal, uint16_t sig_len){
    uint32_t fine_grained_index = 0;

    memcpy(right_buffer, recv_signal + FIX_COARSE_INDEX , sizeof(MIC_TYPE)*CHANNEL_LEN);
    
    if(INIT_FLAG){
           if(last_fine_grained_index <= CONTINUE_TRACKING_RANGE) track_begin_index = 0;
           else track_begin_index = last_fine_grained_index - CONTINUE_TRACKING_RANGE;

           if(last_fine_grained_index +  CONTINUE_TRACKING_RANGE >= CHANNEL_LEN) track_end_index = CHANNEL_LEN - 1;
           else track_end_index = last_fine_grained_index +  CONTINUE_TRACKING_RANGE;

           channel_estimation_freq(right_buffer, right_result, CHIRP_LEN, 1);
     }
     else{
           channel_estimation_freq(right_buffer, right_result, CHIRP_LEN, 1);
     }

}


uint16_t detection_phase_combine(uint16_t I_am_master, uint16_t use_dual){
    uint32_t fine_grained_index = 0;


    if(INIT_FLAG){
      //NRF_LOG_RAW_INFO("track_begin_index = %d, %d, %d,\n ", track_begin_index, track_end_index, previous_coarse_index);


      if(use_dual){
        fine_grained_index = Dual_mic_track(I_am_master);
         if(fine_grained_index == 0){
            NRF_LOG_RAW_INFO("Warnin")
           fine_grained_index = Dual_mic_estimation(I_am_master);
        }
      }
      else{
        fine_grained_index = Single_mic_track(I_am_master);
      }
    }
    else{
      if(use_dual){
        fine_grained_index = Dual_mic_estimation(I_am_master);
      }
      else{
        fine_grained_index = Single_mic_estimation(I_am_master);
      }
    }

    if(CONTINUE_TRACKING){
      last_fine_grained_index = fine_grained_index/2;
      INIT_FLAG = 1;
    }
    
    return fine_grained_index + FIX_COARSE_INDEX * 2;
}