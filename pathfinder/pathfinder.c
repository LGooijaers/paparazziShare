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
 * @file "modules/pathfinder/pathfinder.c"
 * @author Jesse
 * pathfinder Jonny
 */

#include "modules/pathfinder/pathfinder.h"



//
//  pathfinder.c
//
//
//  Created by Jonathan Erskine on 03/04/2018.
//

#include "pathfinder.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int compare(const void *pp0, const void *pp1)
{
    int i0 = **(int **)pp0;
    int i1 = **(int **)pp1;
    if(i0 > i1)return -1;
    if(i0 < i1)return  1;
    return 0;
}

int movement(int nPanels, int nsqPanels, int position, int oldPosition, int north [], int south [], int east [], int west [], int occupied [], int border [], int obstructed [], int corner [], int confidence [] )
{
    int mn = north[position];
    int ms = south[position];
    int me = east[position];
    int mw = west[position];
    int routeN, routeS, routeE, routeW;
    int i;
    int futurePosition;

    if (mn < 0 || mn > (nPanels -1) || obstructed[mn] == 1 || border[mn] == 1 )
    {
        int routeN = 0;
    }
    else{
        int routeN = 1;
    }

    if (ms < 0 || ms > (nPanels -1) || obstructed[ms] == 1 || border[ms] == 1 )
    {
        int routeS = 0;
    }
    else{
        int routeS = 2;
    }

    if (me < 0 || me > (nPanels -1) || obstructed[me] == 1 || border[me] == 1 )
    {
        int routeE = 0;
    }
    else{
        int routeE = 3;
    }

    if (mw < 0 || mw > (nPanels -1) || obstructed[mw] == 1 || border[mw] == 1 )
    {
        int routeW = 0;
    }
    else{
        int routeW = 4;
    }

    int choiceList[] = {mn, ms, me, mw};

    if (routeN != 0)
    {
        if (mn == (2*nsqPanels) + oldPosition)
        {
            confidence[mn] = confidence[mn] - 4;
        }
    }

    if (routeS != 0)
    {
        if (ms == (oldPosition - 2*nsqPanels))
        {
            confidence[ms] = confidence[ms] - 4;
        }
    }

    if (routeE != 0)
    {
        if (ms == (oldPosition + 2))
        {
            confidence[me] = confidence[me] - 4;
        }
    }

    if (routeW != 0)
    {
        if (mw == (oldPosition - 2))
        {
            confidence[mw] = confidence[mw] - 4;
        }
    }

    int confList[] = {confidence[mn],confidence[ms],confidence[me],confidence[mw]};

    int largest = choiceList[0];
    int smallest = choiceList[0];
    for (i = 1; i < 4; i++)
    {
        if (largest < choiceList[i])
        {
            largest = choiceList[i];
        }
        if (smallest > choiceList[i]) {
            smallest = choiceList[i];
        }
    }

    if (smallest == largest) {

        srand(time(NULL));
        int randomIndex = rand() % 4;
        int futurePosition = choiceList[randomIndex];

    } else {
        /*
        int a[4];
        a = confList;
        int b[4];= choiceList;
        int *pa[4];
        size_t i, j, k;
        int ta, tb;
        */
        int *pa[4];
        size_t i, j, k;
        int ta, tb;

        /* create array of pointers to a[] */
        for(i = 0; i < sizeof(confList)/sizeof(confList[0]); i++)
            pa[i] = &confList[i];

        /* sort array of pointers */
        qsort(pa, sizeof(confList)/sizeof(confList[0]), sizeof(pa[0]), compare);

        /* reorder a[] and b[] according to the array of pointers */
        for(i = 0; i < sizeof(confList)/sizeof(confList[0]); i++) {
            if(i != pa[i]-confList){
                ta = confList[i];
                tb = choiceList[i];
                k = i;
                while(i != (j = pa[k]-confList)){
                    confList[k] = confList[j];
                    choiceList[k] = choiceList[j];
                    pa[k] = &confList[k];
                    k = j;
                }
                confList[k] = ta;
                choiceList[k] = tb;
                pa[k] = &confList[k];
            }
        }

        futurePosition = choiceList[3];
        printf("%d\n",choiceList[0]);
        printf("%d\n",confList[0]);
        printf("%d\n",choiceList[1]);
        printf("%d\n",confList[1]);
        printf("%d\n",choiceList[2]);
        printf("%d\n",confList[2]);
        printf("%d\n",choiceList[3]);
        printf("%d\n",confList[3]);
    }
    return futurePosition;
    }

int main(void)
{
    // Set initial conditions
    int nPanels = 25;
    int nsqPanels = sqrt(nPanels);
    int startPosition = nsqPanels + 1;
    int position = startPosition;
    int i;

    // Arrays
    int north [nPanels];
    int south [nPanels];
    int east [nPanels];
    int west [nPanels];
    int occupied [nPanels];
    int border [nPanels];
    int obstructed [nPanels];
    int corner [nPanels];
    int confidence [nPanels];

    // Add borders
    for (i = 0; i < nsqPanels ; ++i ){

        border[i] = 1;
        border[(i*nsqPanels)] = 1;
        border[((i*nsqPanels)-1)] = 1;
    }
    for (i = (nPanels-nsqPanels); i < nPanels ; ++i ){

        border[i] = 1;
    }

    //Add corners
    corner[(nsqPanels+1)] = 1;
    corner[(2*nsqPanels-2)] = 1;
    corner[(nPanels - nsqPanels - 2)] = 1;
    corner[(nPanels - 2*nsqPanels+1)] = 1;

    //Need to create way of inputting if square is obstructed - if ttc < threshold, then panel you're looking at = occupied

    //Add movement functions for each panel
    for ( i = 0; i < nPanels ; ++i ) {

        north[i] = (i + nsqPanels);
        south[i] = (i - nsqPanels);
        east[i] = (i + 1);
        west[i] = (i - 1);
    }

    //Start adding iterations?
    int iteration = 0;
    while (iteration < 100) {
        for ( i = 0; i < nPanels ; ++i ) {
            if ((obstructed[i] = 1)) {
                confidence[i] = 50;
                obstructed[i] = 0;
            }
            if ((border[i] = 1)) {
                confidence[i] = 50;
            }
            if ((corner[i] = 1)) {
                confidence[i] = 10;
            }
        }

        int oldPosition = position;
        confidence[position] = confidence[position] + 10;
        int newPosition = movement(nPanels, nsqPanels, position,oldPosition, north, south, east, west, occupied, border, obstructed, corner, confidence);

        for ( i = 0; i < nPanels ; ++i )
        {
            occupied[i] = 0;
            confidence[i] = confidence[i] - 1;

            if ((confidence[i] < 5))
            {
                confidence[i] = 5;
            }
        }

        position = newPosition;
        occupied[position] = 1;
        printf("%d\n",iteration);
        printf("%d\n",position);
        iteration ++;
    }
    return 0;


}
