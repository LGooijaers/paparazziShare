/*
 * Copyright (C) Roland Meertens
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/orange_avoider/orange_avoider.h"
 * @author Roland Meertens
 * Example on how to use the colours detected to avoid orange pole in the cyberzoo
 */

#ifndef ORANGE_AVOIDER_H
#define ORANGE_AVOIDER_H
#include <inttypes.h>
#include "math/pprz_geodetic_int.h"

//extern uint8_t safeToGoForwards;
//extern float incrementForAvoidance;
//extern uint16_t trajectoryConfidence;
extern void orange_avoider_init(void);
extern void orange_avoider_periodic(void);

_Bool atDestination(uint8_t wp);
void setWaypointRoute(uint8_t wpStart);
void flyRoute(uint8_t WpGoal);

extern uint8_t moveWaypoint(uint8_t waypoint, struct EnuCoor_i *new_coor);
extern uint8_t moveWaypointForward(uint8_t waypoint);

extern uint8_t increaseNavHeading(int32_t *heading, float incrementDegrees);
extern uint8_t chooseRandomIncrementAvoidance(void);
extern void Maneuver(void);
extern void chooseFreeDirection(uint8_t leftArc);
extern void arcCheckObstacles(uint8_t arcThreshold);

#endif

