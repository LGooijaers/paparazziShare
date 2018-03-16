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
 * @file "modules/aa_flight/aa_flight.h"
 * @author Leo
 * flight control functions
 */

#ifndef AA_FLIGHT_H
#define AA_FLIGHT_H

#include "firmwares/rotorcraft/navigation.h"
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

// Waypoint navigation

extern _Bool atDestination(uint8_t wp);
void setWaypointRoute(uint8_t wpStart);
void flyRoute(uint8_t WpGoal);

extern uint8_t moveWpForward(uint8_t goal);
void calcForward(struct EnuCoor_i *new_coor, float distanceMeters);
void moveWp(uint8_t waypoint, struct EnuCoor_i *new_coor);
void changeHeading(int32_t *heading, float incrementDegrees);

#endif
