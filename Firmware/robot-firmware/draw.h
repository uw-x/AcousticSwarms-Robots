/*
 * draw.h
 * maruchi kim
 */

#ifndef _DRAW_H_
#define _DRAW_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <nrfx.h>
#include "gpio.h"

//#include "arm_const_structs.h"

#define GRAPH_WINDOW_HEIGHT              20                                     //!< Graph window height used in draw function.

typedef float float32_t;

void draw_fft_data(float32_t *, uint16_t, uint16_t);

#endif