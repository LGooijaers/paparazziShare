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
 * @file "modules/pathfinder/pathfinder.h"
 * @author Jesse
 * pathfinder Jonny
 */

#ifndef PATHFINDER_H
#define PATHFINDER_H



int compare(const void *pp0, const void *pp1);
int movement(int nPanels, int nsqPanels, int position, int oldPosition, int north [], int south [], int east [], int west [], int occupied [], int border [], int obstructed [], int corner [], int confidence [] );
int main(void);



#endif

