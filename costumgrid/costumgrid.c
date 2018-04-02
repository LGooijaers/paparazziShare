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
 * @file "modules/costumgrid/costumgrid.c"
 * @author Jesse
 * creates a costum grid with x row aolums
 */

#include "modules/costumgrid/costumgrid.h"

#include "math/pprz_algebra_int.h"
#include "state.h"
#include <stdio.h>
#include "firmwares/rotorcraft/navigation.h"

#define N 3

uint8_t wpC1;
uint8_t wpC2;
uint8_t wpC3;
uint8_t wpC4;
uint8_t wpCurrent;

void flyCenter(uint8_t WpGoal)
{
  moveWpForward(WpGoal);
  if (atDestination(wpCurrent))
  {
  	++wpCurrent;
  	VECT3_COPY(navigation_target, waypoints[wpCurrent].enu_i);
  }
}




void createGrid(uint8_t wp1, uint8_t wp2, uint8_t wp3, uint8_t wp4)
{
	struct EnuCoor_i *gridcoor;
	wpC1 = wp1;
	wpC2 = wp2;
	wpC3 = wp3;
	wpC4 = wp4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	gridcoor->x = waypoints[wp1].enu_i.x;
	gridcoor->y = waypoints[wp1].enu_i.y;
	waypoint_set_xy_i(wpC1,  gridcoor->x, gridcoor->y);														// H1
	gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp2].enu_i.x )/N);
	gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp2].enu_i.y )/N);
	waypoint_set_xy_i(wpC2,  gridcoor->x, gridcoor->y);														// H2
	gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp3].enu_i.x )/N);
	gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp3].enu_i.y )/N);
	waypoint_set_xy_i(wpC3,  gridcoor->x, gridcoor->y);														// H3
	gridcoor->x = waypoints[wp1].enu_i.x + ((waypoints[wp4].enu_i.x - waypoints[wp1].enu_i.x )/N);
	gridcoor->y = waypoints[wp1].enu_i.y + ((waypoints[wp4].enu_i.y - waypoints[wp1].enu_i.y )/N);
	waypoint_set_xy_i(wpC4,  gridcoor->x, gridcoor->y);														// H4
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	gridcoor->x = waypoints[wp1].enu_i.x + ((waypoints[wp4].enu_i.x - waypoints[wp1].enu_i.x )*2/N);
	gridcoor->y = waypoints[wp1].enu_i.y + ((waypoints[wp4].enu_i.y - waypoints[wp1].enu_i.y )*2/N);
	waypoint_set_xy_i(wpC1,  gridcoor->x, gridcoor->y);														// H5
	gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp3].enu_i.x - waypoints[wp4].enu_i.x )/N);
	gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp3].enu_i.y - waypoints[wp4].enu_i.y )/N);
	waypoint_set_xy_i(wpC2,  gridcoor->x, gridcoor->y);														// H6
	gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp3].enu_i.x )*2/N);
	gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp3].enu_i.y )*2/N);
	waypoint_set_xy_i(wpC3,  gridcoor->x, gridcoor->y);														// H7
	gridcoor->x = waypoints[wpC3].enu_i.x + ((waypoints[wp2].enu_i.x - waypoints[wp3].enu_i.x )/N);
	gridcoor->y = waypoints[wpC3].enu_i.y + ((waypoints[wp2].enu_i.y - waypoints[wp3].enu_i.y )/N);
	waypoint_set_xy_i(wpC4,  gridcoor->x, gridcoor->y);														// H8
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	++wpC1;++wpC2;++wpC3;++wpC4;
	gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp2].enu_i.x )*2/N);
	gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp2].enu_i.y )*2/N);
	waypoint_set_xy_i(wpC1,  gridcoor->x, gridcoor->y);														// H9
	if (N > 2)
	{
		gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp2].enu_i.x )*3/N);
		gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp2].enu_i.y )*3/N);
		waypoint_set_xy_i(wpC2,  gridcoor->x, gridcoor->y);													// H10
		gridcoor->x = waypoints[wpC2].enu_i.x - ((waypoints[wp2].enu_i.x - waypoints[wp3].enu_i.x )/N);
		gridcoor->y = waypoints[wpC2].enu_i.y - ((waypoints[wp2].enu_i.y - waypoints[wp3].enu_i.y )/N);
		waypoint_set_xy_i(wpC3,  gridcoor->x, gridcoor->y);													// H11
		gridcoor->x = waypoints[wpC2].enu_i.x - ((waypoints[wp2].enu_i.x - waypoints[wp3].enu_i.x )*2/N);
		gridcoor->y = waypoints[wpC2].enu_i.y - ((waypoints[wp2].enu_i.y - waypoints[wp3].enu_i.y )*2/N);
		waypoint_set_xy_i(wpC4,  gridcoor->x, gridcoor->y);													// H12
		++wpC1;++wpC2;++wpC3;++wpC4;
		++wpC1;++wpC2;++wpC3;++wpC4;
		++wpC1;++wpC2;++wpC3;++wpC4;
		++wpC1;++wpC2;++wpC3;++wpC4;
		gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp3].enu_i.x )*3/N);
		gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp3].enu_i.y )*3/N);
		waypoint_set_xy_i(wpC1,  gridcoor->x, gridcoor->y);													// H13
		gridcoor->x = waypoints[wpC1].enu_i.x - ((waypoints[wp3].enu_i.x - waypoints[wp4].enu_i.x )/N);
		gridcoor->y = waypoints[wpC1].enu_i.y - ((waypoints[wp3].enu_i.y - waypoints[wp4].enu_i.y )/N);
		waypoint_set_xy_i(wpC2,  gridcoor->x, gridcoor->y);													// H14
		gridcoor->x = waypoints[wpC1].enu_i.x - ((waypoints[wp3].enu_i.x - waypoints[wp4].enu_i.x )*2/N);
		gridcoor->y = waypoints[wpC1].enu_i.y - ((waypoints[wp3].enu_i.y - waypoints[wp4].enu_i.y )*2/N);
		waypoint_set_xy_i(wpC3,  gridcoor->x, gridcoor->y);													// H15
		gridcoor->x = waypoints[wp1].enu_i.x + ((waypoints[wp4].enu_i.x - waypoints[wp1].enu_i.x )*3/N);
		gridcoor->y = waypoints[wp1].enu_i.y + ((waypoints[wp4].enu_i.y - waypoints[wp1].enu_i.y )*3/N);
		waypoint_set_xy_i(wpC4,  gridcoor->x, gridcoor->y);													// H16
		if (N > 3)
		{
			++wpC1;++wpC2;++wpC3;++wpC4;
			++wpC1;++wpC2;++wpC3;++wpC4;
			++wpC1;++wpC2;++wpC3;++wpC4;
			++wpC1;++wpC2;++wpC3;++wpC4;
			gridcoor->x = waypoints[wp1].enu_i.x + ((waypoints[wp4].enu_i.x - waypoints[wp1].enu_i.x )*4/N);
			gridcoor->y = waypoints[wp1].enu_i.y + ((waypoints[wp4].enu_i.y - waypoints[wp1].enu_i.y )*4/N);
			waypoint_set_xy_i(wpC1,  gridcoor->x, gridcoor->y);														// H17
			gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp3].enu_i.x - waypoints[wp4].enu_i.x )/N);
			gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp3].enu_i.y - waypoints[wp4].enu_i.y )/N);
			waypoint_set_xy_i(wpC2,  gridcoor->x, gridcoor->y);														// H18
			gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp3].enu_i.x - waypoints[wp4].enu_i.x )*2/N);
			gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp3].enu_i.y - waypoints[wp4].enu_i.y )*2/N);
			waypoint_set_xy_i(wpC3,  gridcoor->x, gridcoor->y);														// H19
			gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp3].enu_i.x - waypoints[wp4].enu_i.x )*3/N);
			gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp3].enu_i.y - waypoints[wp4].enu_i.y )*3/N);
			waypoint_set_xy_i(wpC4,  gridcoor->x, gridcoor->y);														// H20
			++wpC1;++wpC2;++wpC3;++wpC4;
			++wpC1;++wpC2;++wpC3;++wpC4;
			++wpC1;++wpC2;++wpC3;++wpC4;
			++wpC1;++wpC2;++wpC3;++wpC4;
			gridcoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp3].enu_i.x )*4/N);
			gridcoor->y = waypoints[wp1].enu_i.y - ((waypoints[wp1].enu_i.y - waypoints[wp3].enu_i.y )*4/N);
			waypoint_set_xy_i(wpC1,  gridcoor->x, gridcoor->y);														// H21
			gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp2].enu_i.x - waypoints[wp3].enu_i.x )/N);
			gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp2].enu_i.y - waypoints[wp3].enu_i.y )/N);
			waypoint_set_xy_i(wpC2,  gridcoor->x, gridcoor->y);														// H22
			gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp2].enu_i.x - waypoints[wp3].enu_i.x )*2/N);
			gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp2].enu_i.y - waypoints[wp3].enu_i.y )*2/N);
			waypoint_set_xy_i(wpC3,  gridcoor->x, gridcoor->y);														// H23
			gridcoor->x = waypoints[wpC1].enu_i.x + ((waypoints[wp2].enu_i.x - waypoints[wp3].enu_i.x )*3/N);
			gridcoor->y = waypoints[wpC1].enu_i.y + ((waypoints[wp2].enu_i.y - waypoints[wp3].enu_i.y )*3/N);
			waypoint_set_xy_i(wpC4,  gridcoor->x, gridcoor->y);														// H24
								// 25 moet er nog in!!!!
		}

	}

}


void navC(uint8_t wp1,uint8_t wp2,uint8_t wp3,uint8_t wp4, uint8_t wp5,uint8_t wp6,uint8_t wp7,uint8_t wp8,uint8_t wp9,uint8_t wp10, uint8_t wp11,uint8_t wp12,uint8_t wp13,uint8_t wp14,uint8_t wp15,uint8_t wp16,uint8_t wp17,uint8_t wp18,uint8_t wp19,uint8_t wp20,uint8_t wp21,uint8_t wp22,uint8_t wp23,uint8_t wp24,uint8_t wp25,uint8_t wpC)
{
	struct EnuCoor_i *centercoor;
	centercoor->x = waypoints[wp1].enu_i.x - ((waypoints[wp1].enu_i.x - waypoints[wp3].enu_i.x )/2);
	centercoor->y = waypoints[wp4].enu_i.y + ((waypoints[wp2].enu_i.y - waypoints[wp4].enu_i.y )/2);
	waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C1
	VECT3_COPY(navigation_target, waypoints[wpC].enu_i);
	wpCurrent = wpC;
	wpC++;
	centercoor->x = waypoints[wp4].enu_i.x - ((waypoints[wp4].enu_i.x - waypoints[wp6].enu_i.x )/2);
	centercoor->y = waypoints[wp5].enu_i.y + ((waypoints[wp3].enu_i.y - waypoints[wp5].enu_i.y )/2);
	waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C2
	wpC++;
	centercoor->x = waypoints[wp3].enu_i.x - ((waypoints[wp3].enu_i.x - waypoints[wp7].enu_i.x )/2);
	centercoor->y = waypoints[wp6].enu_i.y + ((waypoints[wp8].enu_i.y - waypoints[wp6].enu_i.y )/2);
	waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C3
	wpC++;
	centercoor->x = waypoints[wp2].enu_i.x - ((waypoints[wp2].enu_i.x - waypoints[wp8].enu_i.x )/2);
	centercoor->y = waypoints[wp3].enu_i.y + ((waypoints[wp9].enu_i.y - waypoints[wp3].enu_i.y )/2);
	waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C4
	if (N > 2)
		{
		wpC++;
		centercoor->x = waypoints[wp9].enu_i.x - ((waypoints[wp9].enu_i.x - waypoints[wp11].enu_i.x )/2);
		centercoor->y = waypoints[wp8].enu_i.y + ((waypoints[wp10].enu_i.y - waypoints[wp8].enu_i.y )/2);
		waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C5
		wpC++;
		centercoor->x = waypoints[wp8].enu_i.x - ((waypoints[wp8].enu_i.x - waypoints[wp12].enu_i.x )/2);
		centercoor->y = waypoints[wp7].enu_i.y + ((waypoints[wp11].enu_i.y - waypoints[wp7].enu_i.y )/2);
		waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C6
		wpC++;
		centercoor->x = waypoints[wp7].enu_i.x - ((waypoints[wp7].enu_i.x - waypoints[wp13].enu_i.x )/2);
		centercoor->y = waypoints[wp14].enu_i.y + ((waypoints[wp12].enu_i.y - waypoints[wp14].enu_i.y )/2);
		waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C7
		wpC++;
		centercoor->x = waypoints[wp6].enu_i.x - ((waypoints[wp6].enu_i.x - waypoints[wp14].enu_i.x )/2);
		centercoor->y = waypoints[wp15].enu_i.y + ((waypoints[wp7].enu_i.y - waypoints[wp15].enu_i.y )/2);
		waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C8
		wpC++;
		centercoor->x = waypoints[wp5].enu_i.x - ((waypoints[wp5].enu_i.x - waypoints[wp15].enu_i.x )/2);
		centercoor->y = waypoints[wp16].enu_i.y + ((waypoints[wp6].enu_i.y - waypoints[wp16].enu_i.y )/2);
 		waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C9
 		if (N > 5)
 			{
 			wpC++;
 			centercoor->x = waypoints[wp16].enu_i.x - ((waypoints[wp16].enu_i.x - waypoints[wp18].enu_i.x )/2);
 			centercoor->y = waypoints[wp17].enu_i.y + ((waypoints[wp15].enu_i.y - waypoints[wp17].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C10
 			wpC++;
 			centercoor->x = waypoints[wp15].enu_i.x - ((waypoints[wp15].enu_i.x - waypoints[wp19].enu_i.x )/2);
 			centercoor->y = waypoints[wp18].enu_i.y + ((waypoints[wp14].enu_i.y - waypoints[wp18].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C11
 			wpC++;
 			centercoor->x = waypoints[wp14].enu_i.x - ((waypoints[wp14].enu_i.x - waypoints[wp20].enu_i.x )/2);
 			centercoor->y = waypoints[wp19].enu_i.y + ((waypoints[wp13].enu_i.y - waypoints[wp19].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C12
 			wpC++;
 			centercoor->x = waypoints[wp13].enu_i.x - ((waypoints[wp13].enu_i.x - waypoints[wp21].enu_i.x )/2);
 			centercoor->y = waypoints[wp20].enu_i.y + ((waypoints[wp22].enu_i.y - waypoints[wp20].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C13
 			wpC++;
 			centercoor->x = waypoints[wp12].enu_i.x - ((waypoints[wp12].enu_i.x - waypoints[wp22].enu_i.x )/2);
 			centercoor->y = waypoints[wp13].enu_i.y + ((waypoints[wp23].enu_i.y - waypoints[wp13].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C14
 			wpC++;
 			centercoor->x = waypoints[wp11].enu_i.x - ((waypoints[wp11].enu_i.x - waypoints[wp23].enu_i.x )/2);
 			centercoor->y = waypoints[wp12].enu_i.y + ((waypoints[wp24].enu_i.y - waypoints[wp12].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C14
 			wpC++;
 			centercoor->x = waypoints[wp10].enu_i.x - ((waypoints[wp10].enu_i.x - waypoints[wp24].enu_i.x )/2);
 			centercoor->y = waypoints[wp11].enu_i.y + ((waypoints[wp25].enu_i.y - waypoints[wp11].enu_i.y )/2);
 			waypoint_set_xy_i(wpC,  centercoor->x, centercoor->y);															// C16

		}
	}

}














