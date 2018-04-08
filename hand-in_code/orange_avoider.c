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

#ifndef GRID_ROWS
#define GRID_ROWS 3
#endif

#ifndef GRID_COLUMNS
#define GRID_COLUMNS 3
#endif

#ifndef GRID_WEIGHTS
#define GRID_WEIGHTS {1, 2, 3}
#endif

#ifndef GRID_THRESHOLD_1
#define GRID_THRESHOLD_1 1000
#endif

#ifndef GRID_THRESHOLD_2
#define GRID_THRESHOLD_2 1000
#endif

#ifndef GRID_THRESHOLD_3
#define GRID_THRESHOLD_3 1000
#endif

#ifndef VEL_V0
#define VEL_V0 0.05
#endif

#ifndef VEL_V1
#define VEL_V1 0.01
#endif

#ifndef VEL_V2
#define VEL_V2 0.005
#endif


#ifndef VEL_SCALING
#define VEL_SCALING 1.25
#endif

#ifndef VEL_CONF_THRESHOLD_1
#define VEL_CONF_THRESHOLD_1 0
#endif

#ifndef VEL_CONF_THRESHOLD_2
#define VEL_CONF_THRESHOLD_2 2
#endif

#ifndef ANG_0
#define ANG_0 1.5
#endif

#ifndef ANG_1
#define ANG_1 5.0
#endif


#ifndef ANG_2
#define ANG_2 8.0
#endif

#ifndef ANG_3
#define ANG_3 45.0
#endif

float incrementForAvoidance;
// uint8_t safeToGoForwards        = false;
// int tresholdColorCount          = 0.05 * 124800; // 520 x 240 = 124.800 total pixels
// uint16_t trajectoryConfidence   = 1;
// float maxDistance               = 2.25;


#define safetyThreshold 3
#define averageThreshold 2
// #define V0 0.0
// #define V1 0.01
// #define V2 0.05
// #define V3 0.10
#define increment 10.0

float incrementForAvoidance;
uint8_t reason;
// uint8_t V;
uint8_t currentWp;
// uint8_t vision_vector[5];
uint8_t obstaclesPresent[3];
uint8_t midpoint = (GRID_COLUMNS - 1)/2;    // midpoints of the vision columns



/*
 * Initialisation function, setting the colour filter, grid dimensions and pixel threshold per row
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

  // Initialise the grid variables ot make a 3x3 grid
  numRows       = GRID_ROWS;
  numCols       = GRID_COLUMNS;
  numCells		  = GRID_CELLS;

  // Initialise the pixel count threshold for the amount of green pixels
  threshold_cell[0]   = GRID_THRESHOLD_1;
  threshold_cell[1]   = GRID_THRESHOLD_2;
  threshold_cell[2]   = GRID_THRESHOLD_3;

  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}


/*
 * [in]   vision_vector
 * [in]   incrementForAvoidance
 * [out]  waypoint GOAL
 * [out]  waypoint TRAJECTORY
 * [out]  heading change
 * 
 * Function that checks for the amount of obstacles present in side columns of the camera images.
 * It then performs one out of four actions:
 *  - Moves a waypoint forward if safe.
 *  - Slightly changes heading of an obstacle is about to pass sideways.
 *  - Decreases speed and changes heading if obstacle is front but far away.
 *  - Stops flying and performs big heading change.
 */
void orange_avoider_periodic()
{
  VERBOSE_PRINT("Vision vector: [%d, %d, %d] \n", vision_vector[0], vision_vector[1], vision_vector[2]);
  VERBOSE_PRINT("Color counts: \n [%d, %d, %d] \n [%d, %d, %d] \n [%d, %d, %d] \n", color_count_cells[2], color_count_cells[5], color_count_cells[8],
																										 color_count_cells[1], color_count_cells[4], color_count_cells[7],
																										 color_count_cells[0], color_count_cells[3], color_count_cells[6]);

  // Check the amount of green. If this is below a threshold you want to turn a certain amount of degrees
  
  VERBOSE_PRINT("Midpoint: %d \n", vision_vector[midpoint]);
  // Simple decision making tree for movements of the drone.
  if (vision_vector[midpoint] == 0) {   
    // Small heading adjustment if obstacle is next to drone, even though flying straight is free
    if (vision_vector[midpoint-1] > VEL_CONF_THRESHOLD_2 && vision_vector[midpoint+1] == 0) {
      incrementForAvoidance = ANG_0;
      movementHeading(VEL_V0);
    } else if (vision_vector[midpoint+1] > VEL_CONF_THRESHOLD_2 && vision_vector[midpoint-1] == 0) {
      incrementForAvoidance = -ANG_0;
      movementHeading(VEL_V0);
    } else {
      movementNoHeading(VEL_V0);     // keep flying straight
    }

  // Obstacle far away, visible in first row, keep flying straight at lower speed if obstacles far away in all columns (to prevent from turning away early from edge of Cyberzoo)
  } else if (vision_vector[midpoint] == 1) {
    if ((vision_vector[midpoint-1] == 1) && (vision_vector[midpoint] = 1) && (vision_vector[midpoint+1] == 1)) {
      movementNoHeading(VEL_V1);
    } else {
      movementHeading(VEL_V1);
    }

  // Obstacle halfway in vision range, slow down drastically and change heading
  } else if (vision_vector[midpoint] == 2) {
    movementHeading(VEL_V2);

  // Obstacle close by, hover in current positiona nd change heading drastically in direction of last heading change
  } else if (vision_vector[midpoint] == 3) {
    waypoint_set_here_2d(WP_GOAL);
    waypoint_set_here_2d(WP_TRAJECTORY);
    if (incrementForAvoidance < 0.) {
      incrementForAvoidance = -ANG_3;
    } else {
      incrementForAvoidance = ANG_3;
    }
    increase_nav_heading(&nav_heading, incrementForAvoidance);
  
  } else {
    VERBOSE_PRINT("MOVEMENT ERROR"); 
  }
  return;
}





/*
 * Increases the NAV heading. Assumes heading is an INT32_ANGLE. It is bound in this function.
 */
uint8_t increase_nav_heading(int32_t *heading, float incrementDegrees)
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
uint8_t moveWaypointForward(uint8_t waypoint, float distanceMeters)
{
  struct EnuCoor_i new_coor;
  calculateForwards(&new_coor, distanceMeters);
  moveWaypoint(waypoint, &new_coor);
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



/*
 * Check for amount of obstacles present in side columns
 */
uint8_t arcCheckObstacles()
{
  uint8_t count = 0;
	for (uint8_t i=0;i<3;i++)
	{
		if (vision_vector[i] > 0) {
			if (i == midpoint && vision_vector[i] > 0) {
        count--;                                            // Subtract 1 point if there is an object present in the middle column
      }
    count++;
		}
	}
  return count;
}



/*
 * [in]   vision_vector
 * [out]  incrementForAvoidance
 * 
 * Sets the variable 'incrementForAvoidance' based amount and location of the obstacles. 
 * For the current current determination of heading change it is assumed an obstacle is present in front of the drone.
 */
uint8_t chooseIncrementAvoidance()
{
  uint8_t count = arcCheckObstacles();
  // Choose increment to avoid obstacles
 
  // If no obstacles are present in the side columns, set heading change equal to previous direction
  if (count == 0) {
    reason = 0;
    if (incrementForAvoidance < 0.) {
      incrementForAvoidance = -ANG_1;
    } else {
      incrementForAvoidance = ANG_1;
    }

  // If one obstacle is present in the side columns, confidence threshold can be set to to delay response to single object present in side columns
  } else if (count == 1) {
    reason = 1;
    if (vision_vector[midpoint-1] > VEL_CONF_THRESHOLD_1) {
      incrementForAvoidance = ANG_1;
    } else {
      incrementForAvoidance = -ANG_1;
    }

  // If two obstacles are present in the side columns, turns away from nearest object, if at same distance turn direction of last heading change
  } else if (count == 2) {
    reason = 2;
    if (vision_vector[midpoint-1] > vision_vector[midpoint+1]) {
      incrementForAvoidance = ANG_2;
    } else if (vision_vector[midpoint-1] < vision_vector[midpoint+1]) {
      incrementForAvoidance = -ANG_2;
    } else {
      if (incrementForAvoidance < 0.) {
        incrementForAvoidance = -ANG_2;
      } else {
        incrementForAvoidance = ANG_2;
      }
    }
  }
  VERBOSE_PRINT("Set avoidance increment to: %f, reason %d \n", incrementForAvoidance, reason);
  return false;
}

/*
 * Combined movement commands with heading change, changes heading command with incrementForAvoidance value
 */ 
void movementHeading(float velocity) 
{
  increase_nav_heading(&nav_heading, incrementForAvoidance);  
  moveWaypointForward(WP_GOAL, velocity);
  moveWaypointForward(WP_TRAJECTORY, VEL_SCALING * velocity);
  chooseIncrementAvoidance();
  VERBOSE_PRINT("Not changing heading \n");
}


/*
 * Combined movement commands without heading change
 */ 
void movementNoHeading(float velocity) 
{
  moveWaypointForward(WP_GOAL, velocity);
  moveWaypointForward(WP_TRAJECTORY, VEL_SCALING * velocity);
  chooseIncrementAvoidance();
  VERBOSE_PRINT("Changing heading \n");
}
