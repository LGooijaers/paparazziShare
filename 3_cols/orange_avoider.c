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

// #ifndef VEL_V0
// #define VEL_V0 0.05
// #endif

// #ifndef VEL_V1
// #define VEL_V1 0.01
// #endif

// #ifndef VEL_V2
// #define VEL_V2 0.005
// #endif


#ifndef VEL_SCALING
#define VEL_SCALING 1.25
#endif

#ifndef VEL_CONF_THRESHOLD_1
#define VEL_CONF_THRESHOLD_1 1
#endif

#ifndef VEL_CONF_THRESHOLD_2
#define VEL_CONF_THRESHOLD_2 0
#endif

#ifndef ANG_0
#define ANG_0 5.0
#endif

#ifndef ANG_1
#define ANG_1 10.0
#endif


#ifndef ANG_2
#define ANG_2 20.0
#endif

// #ifndef ANG_3
// #define ANG_3 30.0
// #endif

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

  numRows       = GRID_ROWS;
  numCols       = GRID_COLUMNS;
  numCells		  = GRID_CELLS;

  threshold_cell[0]   = GRID_THRESHOLD_1;
  threshold_cell[1]   = GRID_THRESHOLD_2;
  threshold_cell[2]   = GRID_THRESHOLD_3;

  // Initialise random values
  srand(time(NULL));
  chooseRandomIncrementAvoidance();
}

// /*
//  * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
//  */
// void orange_avoider_periodic()
// {
//   VERBOSE_PRINT("Vision vector: [%d, %d, %d, %d, %d] \n", vision_vector[0], vision_vector[1], vision_vector[2], vision_vector[3], vision_vector[4]);
//   VERBOSE_PRINT("Color counts: \n [%d, %d, %d, %d, %d] \n [%d, %d, %d, %d, %d] \n [%d, %d, %d, %d, %d] \n", color_count_cells[2], color_count_cells[5], color_count_cells[8], color_count_cells[11], color_count_cells[14]
// 																										 , color_count_cells[1], color_count_cells[4], color_count_cells[7], color_count_cells[10], color_count_cells[13]
// 																										 , color_count_cells[0], color_count_cells[3], color_count_cells[6], color_count_cells[9], color_count_cells[12]);

//   // Check the amount of orange. If this is above a threshold
//   // you want to turn a certain amount of degrees
//   safeToGoForwards = (
//     vision_vector[midpoint] < safetyThreshold &&
//     vision_vector[midpoint-1] < safetyThreshold &&
//     vision_vector[midpoint+1] < safetyThreshold );
  
//   VERBOSE_PRINT("Safe to go forwards: %d \n", safeToGoForwards);
//   float moveDistance = fmin(maxDistance, 0.05 * trajectoryConfidence);
//   if (safeToGoForwards) {
//     moveWaypointForward(WP_GOAL, moveDistance);
//     moveWaypointForward(WP_TRAJECTORY, 1.25 * moveDistance);
//     nav_set_heading_towards_waypoint(WP_GOAL);
//     // chooseRandomIncrementAvoidance();
//     chooseIncrementAvoidance();
//     trajectoryConfidence += 1;
//   } else {
//     waypoint_set_here_2d(WP_GOAL);
//     waypoint_set_here_2d(WP_TRAJECTORY);
//     increase_nav_heading(&nav_heading, incrementForAvoidance);
//     if (trajectoryConfidence > 5) {
//       trajectoryConfidence -= 4;
//     } else {
//       trajectoryConfidence = 1;
//     }
//   }
//   return;
// }

/*
 * Function that checks it is safe to move forwards, and then moves a waypoint forward or changes the heading
 */
void orange_avoider_periodic()
{
  VERBOSE_PRINT("Vision vector: [%d, %d, %d] \n", vision_vector[0], vision_vector[1], vision_vector[2]);
  VERBOSE_PRINT("Color counts: \n [%d, %d, %d] \n [%d, %d, %d] \n [%d, %d, %d] \n", color_count_cells[2], color_count_cells[5], color_count_cells[8],
																										 color_count_cells[1], color_count_cells[4], color_count_cells[7],
																										 color_count_cells[0], color_count_cells[3], color_count_cells[6]);

  // Check the amount of green. If this is below a threshold
  // you want to turn a certain amount of degrees
  
  VERBOSE_PRINT("Midpoint: %d \n", vision_vector[midpoint]);
  // Simple, cascaded decision making tree for movements of the drone. Still need to implement both change of heading and change of way point in 1 go, look at navigation file for this.
  if (vision_vector[midpoint] == 0) {                              // Flying straight is safe
    movementNoHeading(VEL_V0);
    // // Small heading adjustment if obstacle is next to drone, even though flying straight is free
    // if (vision_vector[midpoint-1] > 1 && vision_vector[midpoint+1] == 0) {
    //   incrementForAvoidance = ANG_0;
    //   movementHeading(VEL_V0);
    // } else if (vision_vector[midpoint+1] > 1 && vision_vector[midpoint-1] == 0) {
    //   incrementForAvoidance = ANG_0;
    //   movementHeading(VEL_V0);
    // } else {   // keep flying straight
    //   movementNoHeading(VEL_V0);
    // }
    
  } else if (vision_vector[midpoint] == 1) { 
    movementHeading(VEL_V1);
  } else if (vision_vector[midpoint] == 2) {
    movementHeading(VEL_V2);
  } else if (vision_vector[midpoint] == 3) {
    waypoint_set_here_2d(WP_GOAL);
    waypoint_set_here_2d(WP_TRAJECTORY);
    if (incrementForAvoidance < 0) {
      incrementForAvoidance = -ANG_3;
    } else {
      incrementForAvoidance = ANG_3;
    }
    increase_nav_heading(&nav_heading, incrementForAvoidance);
    // chooseIncrementAvoidance();
  } else {
    VERBOSE_PRINT("IK BEN EEN RETARD"); 
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
 * Sets the variable 'incrementForAvoidance' based on the presence of obstacles
 */
uint8_t chooseIncrementAvoidance()
{
  uint8_t count = arcCheckObstacles();
  // Choose increment to avoid obstacles
 
  // If no obstacles are present in the side columns
  if (count == 0) {
    reason = 0;
    if (incrementForAvoidance < 0) {
      incrementForAvoidance = -ANG_1;
    } else {
      incrementForAvoidance = ANG_1;
    }

  // If one obstacle is present in the side columns
  } else if (count == 1) {
    reason = 1;
    if (vision_vector[midpoint-1] > VEL_CONF_THRESHOLD_1) {
      incrementForAvoidance = ANG_1;
    } else {
      incrementForAvoidance = -ANG_1;
    }

  // If two obstacles are present in the side columns
  } else if (count == 2) {
    reason = 2;
    if (incrementForAvoidance < 0) {
      incrementForAvoidance = -ANG_2;
    } else {
      incrementForAvoidance = ANG_2;
    }
  }
  VERBOSE_PRINT("Set avoidance increment to: %f, reason %d \n", incrementForAvoidance, reason);
  return false;
}

/*
 * Combined movement commands, changes heading command with incrementForAvoidance value
 */ 
void movementHeading(float velocity) 
{
  increase_nav_heading(&nav_heading, incrementForAvoidance);  
  moveWaypointForward(WP_GOAL, velocity);
  moveWaypointForward(WP_TRAJECTORY, VEL_SCALING * velocity);
  nav_set_heading_towards_waypoint(WP_GOAL);
  chooseIncrementAvoidance();
  // VERBOSE_PRINT("Not changing heading");
}


/*
 * Combined movement commands without changing heading command
 */ 
void movementNoHeading(float velocity) 
{
  moveWaypointForward(WP_GOAL, velocity);
  moveWaypointForward(WP_TRAJECTORY, VEL_SCALING * velocity);
  nav_set_heading_towards_waypoint(WP_GOAL);
  chooseIncrementAvoidance();
  // VERBOSE_PRINT("Changing heading");
}





/*
 * Sets the variable 'incrementForAvoidance' based on the presence of obstacles
 */
// uint8_t chooseIncrementAvoidance()
// {
//   // Choose increment to avoid obstacles
//   if (vision_vector[midpoint-2] == 0 && vision_vector[midpoint-1] == 0 && vision_vector[midpoint+1] == 0 && vision_vector[midpoint-2] == 0) {
//     reason = 0;
//     if (incrementForAvoidance < 0) {
//       incrementForAvoidance = -ANG_1;
//     } else {
//       incrementForAvoidance = ANG_1;
//     }
//   } else if (vision_vector[midpoint-2] == 0 && vision_vector[midpoint-1] == 0 && vision_vector[midpoint+1] == 0 && vision_vector[midpoint-2] == 0) {
//     incrementForAvoidance = -ANG_1;
//     reason =1;
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
//   } else if (vision_vector[midpoint-1] > vision_vector[midpoint+1]){
//     incrementForAvoidance = ANG_1;
//     reason = 2;
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
//   } else if ((vision_vector[midpoint-2] > 1) && (vision_vector[midpoint-2] > vision_vector[midpoint+2])) {
//     incrementForAvoidance = -ANG_2;
//   } else if (vision_vector[midpoint+2] > vision_vector[midpoint-2]) {
//     incrementForAvoidance = ANG_2;
//   } else {      // Can probably remove this entire else statement, just make it use the lanst increment statement, that should be fine, otherwise it starts rotating randomly
//     reason = 3;
    
//     if (incrementForAvoidance < 0) {
//       incrementForAvoidance = -ANG_3;
//     } else {
//       incrementForAvoidance = ANG_3;
//     }
//     VERBOSE_PRINT("Set avoidance increment to: %f\n", incrementForAvoidance);
//     // }
//   }
//   VERBOSE_PRINT("Set avoidance increment to: %f, reason %d \n", incrementForAvoidance, reason);
//   return false;
// }