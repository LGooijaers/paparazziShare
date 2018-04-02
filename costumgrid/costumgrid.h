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
 * @file "modules/costumgrid/costumgrid.h"
 * @author Jesse
 * creates a costum grid with x row aolums
 */

#ifndef COSTUMGRID_H
#define COSTUMGRID_H

#include "firmwares/rotorcraft/navigation.h"
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"


extern void flyCenter(uint8_t WpGoal);

	// create grid x

extern void createGrid(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4);

	// create centers

extern void navC(uint8_t wp1,uint8_t wp2,uint8_t wp3,uint8_t wp4, uint8_t wp5,uint8_t wp6,uint8_t wp7,uint8_t wp8,uint8_t wp9,uint8_t wp10, uint8_t wp11,uint8_t wp12,uint8_t wp13,uint8_t wp14,uint8_t wp15,uint8_t wp16,uint8_t wp17,uint8_t wp18,uint8_t wp19,uint8_t wp20,uint8_t wp21,uint8_t wp22,uint8_t wp23,uint8_t wp24,uint8_t wp25,uint8_t wpC);


#endif
