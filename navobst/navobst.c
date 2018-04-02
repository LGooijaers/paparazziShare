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
 * @file "modules/navobst/navobst.c"
 * @author Jesse
 * nav obstacle using map
 */

#include "modules/navobst/navobst.h"

#include "math/pprz_algebra_int.h"
#include "state.h"
#include <stdio.h>
#include "firmwares/rotorcraft/navigation.h"

uint8_t wpCurrent;
uint8_t wpS;
uint8_t wpC;

#define N 3

void navObst(uint8_t wpGoal, uint8_t c1, uint8_t c2, uint8_t c3, uint8_t c4, uint8_t c5, uint8_t c6, uint8_t c7, uint8_t c8, uint8_t c9)
{
	int mat[3][3] =  {c5, c6, c7, c4, c3, c8, c1, c2, c9};
	int i,j;
	for (i=0;1<4;i++)
	{
		for (j=0;j<4;j++)
			moveWpForward(wpGoal);
			wpCurrent = mat[i][j];
			VECT3_COPY(navigation_target, waypoints[wpCurrent].enu_i);
			if (!atDestination(wpCurrent))
			{
		  	return false;
			}
	}

}


void setCenters(uint8_t wp1,uint8_t wp2,uint8_t wp3,uint8_t wp4)
{
	struct EnuCoor_i *gridcoor;
	int j;
	for (j=0;j<(N+1);j++)
	{
			wpS = wp1;
			wpC = wp4;
			wpC++;
			gridcoor->x = waypoints[wpS].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp2].enu_i.x )*j/N);
			gridcoor->y = waypoints[wpS].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp2].enu_i.y )*j/N);
			waypoint_set_xy_i(wpC,  gridcoor->x, gridcoor->y);

	}
	return false;
}
