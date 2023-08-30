/*
 * draw.c
 * maruchi kim
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <nrfx.h>
#include "app_util_platform.h"
// #include "bsp.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
//#include "arm_const_structs.h"

#include "app_timer.h"

#include "nrf_pdm.h"
#include "nrfx_pdm.h"
#include "nrf_assert.h"

#include "draw.h"

#include "draw.h"

/**
 * @brief Function for drawing line with given width.
 * @param[in] line_width Line width.
 */
static void draw_line(uint16_t line_width)
{
    uint32_t i;
    char     line[line_width + 1];

    for (i = 0; i < line_width; i++)
    {
        line[i] = '-';
    }
    line[line_width] = 0;
    // NRF_LOG_RAW_INFO("%s\r\n", nrf_log_push(line));
    NRF_LOG_RAW_INFO("%s\r\n", line);
}

/**
 * @brief Function for drawing ASCII data processed by FFT function.
 * @param[in] p_input_data Pointer to FFT data array.
 * @param[in] data_size    FFT array size.
 * @param[in] chart_height Drawing chart height.
 */
void draw_fft_data(float32_t * p_input_data, uint16_t data_size, uint16_t chart_height)
{
    uint32_t  graph_y, graph_x;
    float32_t curr_drawing_val;
    float32_t curr_percent;
    float32_t max_value;
    uint32_t  max_val_index;
    char      tmp_str[data_size + 1];

    // Search FFT max value in input array.
    arm_max_f32(p_input_data, data_size, &max_value, &max_val_index);

    // Draw graph. Put space if number is less than currently processed, put '|' character otherwise.
    for (graph_y = chart_height; graph_y > 0; graph_y--)
    {
        curr_percent = ((graph_y - 1) / (chart_height * 1.f));
        curr_drawing_val = max_value * curr_percent;
        for (graph_x = 0; graph_x < data_size; graph_x++)
        {
            if (p_input_data[graph_x] > curr_drawing_val)
            {
                tmp_str[graph_x] = '|';
            } else
            {
                tmp_str[graph_x] = ' ';
            }
        }
        tmp_str[data_size] = 0;
        // NRF_LOG_RAW_INFO("%s\r\n", NRF_LOG_PUSH(tmp_str));
        NRF_LOG_RAW_INFO("%s\r\n", tmp_str);
        NRF_LOG_FLUSH();
    }

    draw_line(data_size);
}