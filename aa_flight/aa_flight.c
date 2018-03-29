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
 * @file "modules/aa_flight/aa_flight.c"
 * @author Leo
 * first flight module
 */

#include "modules/aa_flight/aa_flight.h"
#include "firmwares/rotorcraft/navigation.h"
#include "math/pprz_algebra_int.h"
#include "state.h"
#include <stdio.h>

#define safetyThreshold 1
#define averageThreshold 2
#define arcCloseThreshold 2
#define arcFarThreshold 3
#define V1 0.01
#define V2 0.05
#define V3 0.1
#define increment 10.0

float incrementForAvoidance;
uint8_t V;
uint8_t currentWp;
uint8_t ttc_vector[5];
uint8_t obstaclesPresent[5];

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

/*********************************************/
/********** Move local waypoint **************/
/*********************************************/
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
}

/* Sets waypoint 'waypoint' to the coordinates of 'new_coor' */
void moveWp(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
}

/*********************************************/
/************** navigation *******************/
/*********************************************/
void Object_avoider_periodic()
{
	//vision(); //Do vision algorithm
	_Bool safeToGo = ttc_vector[2] < safetyThreshold; //2 is the middle front arc
	_Bool safeToGoForward = ttc_vector[2] < averageThreshold;

	if (!safeToGo) {
		waypoint_set_here_2d(5);//waypoint_set_here_2d(WP_GOAL);
		changeHeading(&nav_heading, incrementForAvoidance);
	}

	if (safeToGoForward) {
		flyRoute(5);//flyRoute(WP_GOAL);
		createRandomIncrementAvoidance();
	}
	else {
		V = V2; //Slow down
		//TODO create arcCheckObstacles function
		arcCheckObstacles(&ttc_vector,arcCloseThreshold);
		if (obstaclesPresent[1] == obstaclesPresent[3]){
			if (obstaclesPresent[1] == 0){ 							// no objects to either close side arcs
				changeHeading(&nav_heading, incrementForAvoidance); //TODO implement intelligent decision system
			}
			else {													//objects in both close side arcs
				V = V3;
				Maneuver(&ttc_vector);
			}
		}
		else{														//an object in one close side arc
			chooseFreeDirection(obstaclesPresent[1]);				// choose the direction without an object
		}
	}
	return;
}

void Maneuver(uint8_t *pTtc_vector)
{
	if (obstaclesPresent[0] == obstaclesPresent[4]){
		if (obstaclesPresent[0] == 0){						// both sides are clear of obstacles
			changeHeading(&nav_heading, incrementForAvoidance); //TODO implement intelligent decision system
		}
		else {
			waypoint_set_here_2d(5);//waypoint_set_here_2d(WP_GOAL);						//no safe path hence stop in turn
			changeHeading(&nav_heading, incrementForAvoidance);
		}
	}
    else{
		chooseFreeDirection(obstaclesPresent[1]);				// choose the direction without an object
    	// farObject(pTtc_vector)
    }
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

void chooseFreeDirection(uint8_t leftArc)
{
	if (leftArc == 0){						// left arc is free
		incrementForAvoidance = -increment;	// turn left
	} else {								// right arc is free
		incrementForAvoidance = increment;	// turn right
	}
}

uint8_t createRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = increment;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -increment;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}

/*********************************************/
/************* Spare functions ***************/
/*********************************************/


