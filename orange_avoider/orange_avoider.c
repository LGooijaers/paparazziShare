/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.c"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#include "modules/orange_avoider/orange_avoider.h"
#include "modules/computer_vision/colorfilter.h"
#include "firmwares/rotorcraft/navigation.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "state.h"
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>


#define ORANGE_AVOIDER_VERBOSE TRUE

#define PRINT(string,...) fprintf(stderr, "[orange_avoider->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#if ORANGE_AVOIDER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef ORANGE_AVOIDER_LUM_MIN
#define ORANGE_AVOIDER_LUM_MIN 41
#endif

#ifndef ORANGE_AVOIDER_LUM_MAX
#define ORANGE_AVOIDER_LUM_MAX 183
#endif

#ifndef ORANGE_AVOIDER_CB_MIN
#define ORANGE_AVOIDER_CB_MIN 53
#endif

#ifndef ORANGE_AVOIDER_CB_MAX
#define ORANGE_AVOIDER_CB_MAX 121
#endif

#ifndef ORANGE_AVOIDER_CR_MIN
#define ORANGE_AVOIDER_CR_MIN 134
#endif

#ifndef ORANGE_AVOIDER_CR_MAX
#define ORANGE_AVOIDER_CR_MAX 249
#endif


/*uint8_t safeToGoForwards        = false;
int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
float incrementForAvoidance;
uint16_t trajectoryConfidence   = 1;
float maxDistance               = 2.25;
*/

#define safetyThreshold 3
#define averageThreshold 2
#define V1 0.01
#define V2 0.05
#define V3 0.1
#define increment 10.0

float incrementForAvoidance;
uint8_t V;
uint8_t currentWp;
uint8_t vision_vector[5];
uint8_t obstaclesPresent[5];

/*********************************************/
/************* Initialization ****************/
/*********************************************/
/*
 * Initialisation function, setting the colour filter, random seed and incrementForAvoidance
 */
void orange_avoider_init()
{
  // Initialise the variables of the colorfilter to accept orange
  color_lum_min = ORANGE_AVOIDER_LUM_MIN;
  color_lum_max = ORANGE_AVOIDER_LUM_MAX;
  color_cb_min  = ORANGE_AVOIDER_CB_MIN;
  color_cb_max  = ORANGE_AVOIDER_CB_MAX;
  color_cr_min  = ORANGE_AVOIDER_CR_MIN;
  color_cr_max  = ORANGE_AVOIDER_CR_MAX;
  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}


/*********************************************/
/**************** Periodic *******************/
/*********************************************/
/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic()
{
	_Bool safeToGo = vision_vector[2] < safetyThreshold; //2 is the middle front arc
	_Bool safeToGoForward = vision_vector[2] < averageThreshold;

	if (!safeToGo) {
		waypoint_set_here_2d(5);	//TODO waypoint_set_here_2d(WP_GOAL);
		increaseNavHeading(&nav_heading, incrementForAvoidance);
	}

	if (safeToGoForward) {
		flyRoute(5);				//TODO flyRoute(WP_GOAL);
		chooseRandomIncrementAvoidance();
	}
	else {
		V = V2; //Slow down
		//TODO create arcCheckObstacles function
		arcCheckObstacles(2);
		if (obstaclesPresent[1] == obstaclesPresent[3]){
			if (obstaclesPresent[1] == 0){ 							// no objects to either close side arcs
				increaseNavHeading(&nav_heading, incrementForAvoidance); //TODO implement intelligent decision system
			}
			else {													//objects in both close side arcs
				V = V3;
				Maneuver();
			}
		}
		else{														//an object in one close side arc
			chooseFreeDirection(obstaclesPresent[1]);				// choose the direction without an object
		}
	}
	return;
}

/*********************************************/
/****************** General ******************/
/*********************************************/

_Bool atDestination(uint8_t wp)
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
	moveWaypointForward(WpGoal);
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
/*
 * Calculates coordinates of a distance of 'distanceMeters' forward w.r.t. current position and heading
 */
static uint8_t calculateForwards(struct EnuCoor_i *new_coor, float distanceMeters)
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
  return false;
}

/*
 * Sets waypoint 'waypoint' to the coordinates of 'new_coor'
 */
uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor)
{
  VERBOSE_PRINT("Moving waypoint %d to x:%f y:%f\n", waypoint, POS_FLOAT_OF_BFP(new_coor->x),
                POS_FLOAT_OF_BFP(new_coor->y));
  waypoint_set_xy_i(waypoint, new_coor->x, new_coor->y);
  return false;
}


/*
 * Calculates coordinates of distance forward and sets waypoint 'waypoint' to those coordinates
 */
uint8_t moveWaypointForward(uint8_t waypoint)
{
	struct EnuCoor_i new_coor;
	calculateForwards(&new_coor, V);
	moveWaypoint(waypoint, &new_coor);
	return false;
}

/*********************************************/
/************** Navigation *******************/
/*********************************************/

/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increaseNavHeading(int32_t *heading, float incrementDegrees)
{
  struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
  int32_t newHeading = eulerAngles->psi + ANGLE_BFP_OF_REAL(RadOfDeg(incrementDegrees));
  // Check if your turn made it go out of bounds...
  INT32_ANGLE_NORMALIZE(newHeading); // HEADING HAS INT32_ANGLE_FRAC....
  *heading = newHeading;
  VERBOSE_PRINT("Increasing heading to %f\n", DegOfRad(ANGLE_FLOAT_OF_BFP(*heading)));
  return false;
}

/*
 * Sets the variable 'incrementForAvoidance' randomly positive/negative
 */
uint8_t chooseRandomIncrementAvoidance()
{
  // Randomly choose CW or CCW avoiding direction
  int r = rand() % 2;
  if (r == 0) {
    incrementForAvoidance = 10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  } else {
    incrementForAvoidance = -10.0;
    VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
  }
  return false;
}


void Maneuver()
{
	arcCheckObstacles(2);
	if (obstaclesPresent[0] == obstaclesPresent[4]){
		if (obstaclesPresent[0] == 0){						// both sides are clear of obstacles
			increaseNavHeading(&nav_heading, incrementForAvoidance); //TODO implement intelligent decision system
		}
		else {
			waypoint_set_here_2d(5);//waypoint_set_here_2d(WP_GOAL);						//no safe path hence stop in turn
			increaseNavHeading(&nav_heading, incrementForAvoidance);
		}
	}
    else{
		chooseFreeDirection(obstaclesPresent[1]);				// choose the direction without an object
    	// farObject(pTtc_vector)
    }
}

void chooseFreeDirection(uint8_t leftArc)
{
	if (leftArc == 0){						// left arc is free
		incrementForAvoidance = -increment;	// turn left
	} else {								// right arc is free
		incrementForAvoidance = increment;	// turn right
	}
}

void arcCheckObstacles(uint8_t arcThreshold)
{
	for (uint8_t i=0;i<5;i++)
	{
		if (vision_vector[i] >= arcThreshold) {
			obstaclesPresent[i] = 1;
		} else {
			obstaclesPresent[i] = 0;
		}
	}
}


