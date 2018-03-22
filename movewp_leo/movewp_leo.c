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
 * @file "modules/movewp_leo/movewp_leo.c"
 * @author Jesse
 * move wp file van Leo
 */

#include "modules/movewp_leo/movewp_leo.h"


#include "math/pprz_algebra_int.h"
#include "state.h"
#include <stdio.h>
#include "firmwares/rotorcraft/navigation.h"

/* Definitions */
#define CARROT_DIST (12 << INT32_POS_FRAC)
#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

uint8_t currentWp;
#define V1 0.1
#define V2 0.05
#define V3 0.01

/* Functions */

extern _Bool atDestination(uint8_t wp)
{
  float dist = get_dist2_to_waypoint(wp);
  if (dist < 0.2) {return true;}
  else {return false;}
}

void setWaypointRoute(uint8_t wpStart)
{
  VECT3_COPY(navigation_target, waypoints[wpStart].enu_i);
  currentWp = wpStart;
}

void flyRoute(uint8_t WpGoal)
{
  moveWpForward(WpGoal);
  nav_set_heading_towards_waypoint(currentWp);
  if (atDestination(currentWp))
  {
	++currentWp;
	VECT3_COPY(navigation_target, waypoints[currentWp].enu_i);
  }
}

/* Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates */
uint8_t moveWpForward(uint8_t goal)
{
  struct EnuCoor_i new_coor;
  calcForward(&new_coor, V2);
  moveWp(goal, &new_coor);
  return false;
}

/* Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading */
void calcForward(struct EnuCoor_i *new_coor, float distanceMeters)
{
  struct EnuCoor_i *pos             = stateGetPositionEnu_i(); // Get your current position
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  // Calculate the sine and cosine of the heading the drone is keeping
  float sin_heading                 = sinf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  float cos_heading                 = cosf(ANGLE_FLOAT_OF_BFP(eulerAngles->psi));
  // Now determine where to place the waypoint you want to go to
  new_coor->x                       = pos->x + POS_BFP_OF_REAL(sin_heading * (distanceMeters));
  new_coor->y                       = pos->y + POS_BFP_OF_REAL(cos_heading * (distanceMeters));
  VERBOSE_PRINT("Calculated %f m forward position. x: %f  y: %f based on pos(%f, %f) and heading(%f)\n", distanceMeters,
                POS_FLOAT_OF_BFP(new_coor->x), POS_FLOAT_OF_BFP(new_coor->y), POS_FLOAT_OF_BFP(pos->x), POS_FLOAT_OF_BFP(pos->y),
                DegOfRad(ANGLE_FLOAT_OF_BFP(eulerAngles->psi)) );
}

/* Sets waypoint 'waypoint' to the coordinates of 'new_coor' */
void moveWp(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
}

/* Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function. */
void changeHeading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
}
