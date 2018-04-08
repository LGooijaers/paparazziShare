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
 * @file modules/computer_vision/colorfilter.c
 */

// Own header
#include "modules/computer_vision/colorfilter.h"
#include <stdio.h>

#include "modules/computer_vision/lib/vision/image.h"

#ifndef COLORFILTER_FPS
#define COLORFILTER_FPS 0       ///< Default FPS (zero means run at camera fps)
#endif
PRINT_CONFIG_VAR(COLORFILTER_FPS)


#ifndef COLORFILTER_SEND_OBSTACLE
#define COLORFILTER_SEND_OBSTACLE FALSE    ///< Default sonar/agl to use in opticflow visual_estimator
#endif
PRINT_CONFIG_VAR(COLORFILTER_SEND_OBSTACLE)

struct video_listener *listener = NULL;

// Color filter settings
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

// Dimensions of the vision grid
uint8_t numRows       = 3;
uint8_t numCols       = 3;
uint8_t numCells      = 9;

// Threshold for color count per row of cells, and the weights assigned from top to bottom row
uint16_t threshold_cell[3];
uint8_t weights[3] =  {1, 2, 3}; 

// Results
int color_count_cells[9];
int *cnt_cells = color_count_cells;

int vision_vector[3];
int *cnt = vision_vector;

#include "subsystems/abi.h"




/*
 * [in]   img
 * [out]  vision_vector
 * 
 * Function that assigns weights based on the grid columns on the input image. The value depends on which cell(s) in the respective column have a pixelcount below the threshold.
 * If the amount of green pixels in a cell drops below a threshold it is assigned a weight, as this indicates and object is present blocking the green of the floor.
 * The rows towards the bottom of the image are assigned the highest weight decreasing to the top side of the image because object in the bottom rows are closer by than in the higher rows.
 * Each column can only have one value, the highest weight in its cells.
 * 
 * The function returns an integer array vision_vector with its length equal to the amount of columns, e.q. vision_vector[3] = {1, 3, 3}
 */ 
struct image_t *section_counter(struct image_t *img)
{
    image_yuv422_colorfilt_cells(img, img, 
                               color_lum_min, color_lum_max, 
                               color_cb_min, color_cb_max, 
                               color_cr_min, color_cr_max,
                               cnt_cells);


  // Assign distance weight to the cells if the number of green pixels in the cell drops below a threshold 
  for(int i = 0; i < numCols; i++){
    *(cnt + i) = 0;
    for(int j = 0; j < numRows; j++){
      if(*(cnt_cells + (i*3 + 2 - j)) < threshold_cell[j]){
        *(cnt + i) = weights[j]; 
      }
    }
  }   
  return img;   // section_counter did not make a new image                            
}


void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, section_counter, COLORFILTER_FPS);
}