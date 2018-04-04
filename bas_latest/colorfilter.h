/*
 * Copyright (C) 2015
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/computer_vision/colorfilter.h
 */

#ifndef COLORFILTER_CV_PLUGIN_H
#define COLORFILTER_CV_PLUGIN_H

#include <stdint.h>
#include "modules/computer_vision/cv.h"

// Module functions
extern void colorfilter_init(void);
struct image_t *section_counter(struct image_t *img);

extern uint8_t color_lum_min;
extern uint8_t color_lum_max;

extern uint8_t color_cb_min;
extern uint8_t color_cb_max;

extern uint8_t color_cr_min;
extern uint8_t color_cr_max;

extern uint8_t numRows;
extern uint8_t numCols;
extern uint8_t numCells;

// extern uint16_t rowArray[3];
// extern uint16_t columnArray[5];

extern uint16_t threshold_cell;

// extern uint8_t weightOne;
// extern uint8_t weightTwo;
// extern uint8_t weightThree;

extern uint8_t weights[3];

extern int color_count_cells[15];
extern int *cnt_cells;

extern int vision_vector[5];
extern int *cnt;

// extern uint8_t obstaclesPresent;

extern struct video_listener *listener;

#endif /* COLORFILTER_CV_PLUGIN_H */
