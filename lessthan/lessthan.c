/*
 * Copyright (C) Leo
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
 * @file "modules/lessthan/lessthan.c"
 * @author Leo
 * test module
 */

/* Includes */
#include "modules/lessthan/lessthan.h"

/* Definitions */
#define CARROT_DIST (12 << INT32_POS_FRAC)

struct EnuCoor_i navigation_target;
struct EnuCoor_i navigation_carrot;

// nav_route variables
struct EnuCoor_i nav_segment_start, nav_segment_end;
struct EnuCoor_i route_start, route_end;

/* Functions */
extern _Bool atDestination(uint8_t wp)
{
  float dist = get_dist2_to_waypoint(wp);
  if (dist < 0.25) {return true;}
  else {return false;}
}

void setWaypointRoute(uint8_t wpStart, uint8_t wpEnd)
{
  VECT3_COPY(route_start, waypoints[wpStart].enu_i);
  VECT3_COPY(route_end, waypoints[wpEnd].enu_i);
}

void flyRoute()
{
  nav_route(&route_start, &route_end);
}

void flyToWaypoint(float x, float y)
{
  struct FloatVect2 target = {x, y};
  struct FloatVect2 pos_diff;
  VECT2_DIFF(pos_diff, target, *stateGetPositionEnu_f());
  // don't change heading if closer than 0.5m to target
  if (VECT2_NORM2(pos_diff) > 0.25) 
  {
    float heading_f = atan2f(pos_diff.x, pos_diff.y);
    nav_heading = ANGLE_BFP_OF_REAL(heading_f);
  }
}

void setHeadingToWaypoint(uint8_t wp)
{
  flyToWaypoint(WaypointX(wp), WaypointY(wp));
}
