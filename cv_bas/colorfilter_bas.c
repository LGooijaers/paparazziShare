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

// Filter Settings - Change these to the values Wessel found for the color filter
uint8_t color_lum_min = 105;
uint8_t color_lum_max = 205;
uint8_t color_cb_min  = 52;
uint8_t color_cb_max  = 140;
uint8_t color_cr_min  = 180;
uint8_t color_cr_max  = 255;

uint8_t numRows       = 3;
uint8_t numCols       = 5;

iunt16_t threshold_cell = 5000;

uint8_t weights       = {1, 2, 3};


// Result
int color_count_cells[numRows * numCols] = { 0 };
int *cnt_cells;
cnt_cells = &color_count_cells;

uint8_t color_count[numRows * numCols] = { 0 };
int *cnt;
cnt = &color_count;


#include "subsystems/abi.h"


struct image_t *section_counter(struct image_t *img)
{
  // Filter the cells of the image for the selected color
  image_yuv422_colorfilt_cells(img, img,
                                color_lum_min, color_lum_max,
                                color_cb_min, color_cb_max,
                                color_cr_min, color_cr_max,
                                rowArray, columnArray,
                                numRows, numCols,
                                *cnt_cells
                                );

  
  // Assign distance weight to the cells if the number of green pixels in the cell drops below a threshold 
  for(int i = 0; < numRows; i += 1){
    for(int j = 0; < numCols; j += 1){
      if(*cnt_cells[i*5 + j] < threshold_cell){
        *cnt[i*5 + j] = weights[i];
      }
      else{
        *cnt[i*5 + j] = 0;
      }
    }
  }   
  return img;   // section_counter did not make a new image                            
}


void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, section_counter, COLORFILTER_FPS);
}






// // Function
// // struct image_t *colorfilter_func(struct image_t *img);
// struct image_t *colorfilter_func(struct image_t *img)
// {
//   // Filter
//   image_yuv422_colorfilt_cells(img, img,
//                                        color_lum_min, color_lum_max,
//                                        color_cb_min, color_cb_max,
//                                        color_cr_min, color_cr_max,
//                                        rowArray, columnArray,
//                                        numRows, numCols,
//                                        *cnt
//                                       );


//   if (COLORFILTER_SEND_OBSTACLE) {
//     if (color_count > 20)
//     {
//       AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 1.f, 0.f, 0.f);
//     }
//     else
//     {
//       AbiSendMsgOBSTACLE_DETECTION(OBS_DETECTION_COLOR_ID, 10.f, 0.f, 0.f);
//     }
//   }

//   return img; // Colorfilter did not make a new image
// }

void colorfilter_init(void)
{
  listener = cv_add_to_device(&COLORFILTER_CAMERA, colorfilter_func, COLORFILTER_FPS);
}
