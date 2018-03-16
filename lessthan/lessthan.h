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
 * @file "modules/lessthan/lessthan.h"
 * @author Leo
 * test module
 */

#ifndef LESSTHAN_H
#define LESSTHAN_H

#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_algebra_int.h"
#include "state.h"

// Waypoint navigation
extern _Bool atDestination(uint8_t wp);
void setWaypointRoute(uint8_t wpStart, uint8_t wpEnd);
void flyRoute();
void flyToWaypoint(float x, float y);
void setHeadingToWaypoint(uint8_t wp);

#endif

