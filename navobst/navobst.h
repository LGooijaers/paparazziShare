/*
 * Copyright (C) Jesse
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/navobst/navobst.h"
 * @author Jesse
 * nav obstacle using map
 */

#ifndef NAVOBST_H
#define NAVOBST_H

#include "firmwares/rotorcraft/navigation.h"
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"



extern void navObst(uint8_t wpGoal, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7, uint8_t c8, uint8_t c9);
extern void setCenters(uint8_t wp1,uint8_t wp2,uint8_t wp3,uint8_t wp4);

#endif
